import os
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import re
from pathlib import Path
from datetime import datetime

import cv2
import serial
import rclpy
from rclpy.node import Node
from ultralytics import YOLO


PORT = "/dev/ttyACM0"
BAUD = 115200
CAM_ID = 0
SPEED = 70

MODEL_PATH = str(Path.home() / "ros2_car_ws" / "yolov8n.pt")
EVENT_DIR = Path.home() / "ros2_car_ws" / "patrol_events"
LOG_FILE = EVENT_DIR / "events.log"

PERSON_CLOSE_HEIGHT_RATIO = 0.45
OBSTACLE_CLOSE_AREA_RATIO = 0.05

CONF_THRES = 0.45
COMMAND_INTERVAL = 1.5

CLOSE_CONFIRM_FRAMES = 4
STOPPED_CLEAR_FRAMES = 5

OBSTACLE_NAMES = {
    "bottle",
    "chair",
    "backpack",
    "handbag",
    "suitcase",
    "sports ball",
    "cup",
    "box",
}



LATEST_JPEG = None
JPEG_LOCK = threading.Lock()


class VideoStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        global LATEST_JPEG

        if self.path == "/" or self.path == "/index.html":
            page = """
            <html>
            <head>
                <meta charset="utf-8">
                <title>ROS2 YOLO Camera</title>
                <style>
                    body { background:#111; color:white; text-align:center; font-family:Arial; }
                    img { max-width:95vw; max-height:85vh; border:2px solid #555; }
                </style>
            </head>
            <body>
                <h2>ROS2 YOLO Running Camera</h2>
                <img src="/stream.mjpg">
            </body>
            </html>
            """
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(page.encode("utf-8"))
            return

        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            with JPEG_LOCK:
                frame = LATEST_JPEG

            if frame is not None:
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                except Exception:
                    break

            time.sleep(0.05)


def start_video_server():
    server = HTTPServer(("0.0.0.0", 8090), VideoStreamHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print("[video] stream server started: http://0.0.0.0:8090")


class PatrolNode(Node):
    def __init__(self):
        start_video_server()
        super().__init__("yolo_auto_patrol_node")

        EVENT_DIR.mkdir(parents=True, exist_ok=True)

        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(1.0)

        self.cap = cv2.VideoCapture(CAM_ID)
        if not self.cap.isOpened():
            raise RuntimeError("/dev/video0 open failed")

        self.model = YOLO(MODEL_PATH)

        self.state = "PATROL"
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.close_count = 0
        self.stopped_clear_count = 0
        self.stopped_printed = False
        self.last_print_time = 0.0
        self.us_blocked = False
        self.us_distance_cm = None
        self.last_us_poll_time = 0.0

        self.get_logger().info("[patrol] init serial and ultrasonic guard")

        self.send_cmd("car_stop", force=True)
        self.send_cmd("us_guard_start", force=True)
        self.send_cmd("us_guard_state", force=True)

    def send_cmd(self, cmd, force=False):
        now = time.time()

        if (not force) and self.last_cmd == cmd and (now - self.last_cmd_time) < COMMAND_INTERVAL:
            return

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        self.ser.write((cmd + "\r\n").encode())
        time.sleep(0.05)

        try:
            out = self.ser.read_all().decode(errors="ignore")
            for line in out.splitlines():
                line = line.strip()
                if not line:
                    continue
                if line.startswith("msh"):
                    continue
                if line == cmd:
                    continue
                if line.startswith("car ") and "speed=" in line:
                    continue
                if line.startswith("[us_guard]") or line.startswith("[safe]"):
                    self.update_us_from_line(line)
                    print(line)
        except Exception:
            pass

        if force or not cmd.startswith("car_forward"):
            print("[car] send", cmd)

        self.last_cmd = cmd
        self.last_cmd_time = now

    def update_us_from_line(self, line):
        """解析 PSoC62 超声波 us_guard 输出。"""
        try:
            if "[safe]" in line and "ultrasonic" in line:
                m = re.search(r"ultrasonic\s+([0-9.]+)\s*cm", line)
                if m:
                    self.us_distance_cm = float(m.group(1))
                    self.us_blocked = True
                return

            if "[us_guard]" not in line:
                return

            m = re.search(r"blocked=(\d+)", line)
            if m:
                self.us_blocked = (m.group(1) == "1")

            m = re.search(r"last_distance=([0-9.]+)", line)
            if m:
                self.us_distance_cm = float(m.group(1))

            m = re.search(r"obstacle\s+([0-9.]+)\s*cm", line)
            if m:
                self.us_distance_cm = float(m.group(1))
                self.us_blocked = True

            m = re.search(r"clear\s+([0-9.]+)\s*cm", line)
            if m:
                self.us_distance_cm = float(m.group(1))
                self.us_blocked = False

        except Exception:
            pass

    def poll_ultrasonic(self, force=False):
        """
        主动查询超声波距离。
        不走 send_cmd，避免把 last_cmd 改成 us_guard_state，影响 car_forward 节流。
        """
        now = time.time()
        if (not force) and (now - self.last_us_poll_time) < 0.5:
            return

        self.last_us_poll_time = now

        try:
            self.ser.write(b"us_guard_state\r\n")
            time.sleep(0.05)
            out = self.ser.read_all().decode(errors="ignore")
            for line in out.splitlines():
                line = line.strip()
                if not line:
                    continue
                if line.startswith("[us_guard]") or line.startswith("[safe]"):
                    self.update_us_from_line(line)
                    print(line)
        except Exception:
            pass


    def update_video_stream(self, frame, info=None):
        global LATEST_JPEG

        try:
            show = frame.copy()

            if info is not None:
                person_count = info.get("person_count", 0)
                cargo_count = info.get("cargo_count", info.get("obstacle_count", 0))
                us_distance = info.get("ultrasonic_distance_cm", None)

                # 左上角信息框，避免文字被画面裁掉
                cv2.rectangle(show, (10, 10), (330, 115), (0, 0, 0), -1)

                cv2.putText(show, f"person: {person_count}", (20, 38),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                cv2.putText(show, f"cargo: {cargo_count}", (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if us_distance is not None:
                    us_text = f"ultrasonic: {us_distance:.1f} cm"
                else:
                    us_text = "ultrasonic: invalid"

                cv2.putText(show, us_text, (20, 102),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            ok, jpg = cv2.imencode(".jpg", show)
            if ok:
                with JPEG_LOCK:
                    LATEST_JPEG = jpg.tobytes()

        except Exception as e:
            print("[video] update failed:", e)


    def detect_frame(self, frame):
        h, w = frame.shape[:2]
        frame_area = h * w

        results = self.model(frame, imgsz=640, conf=CONF_THRES, verbose=False)
        r = results[0]

        person_count = 0
        obstacle_count = 0

        max_person_h_ratio = 0.0
        max_obstacle_area_ratio = 0.0
        max_person_aspect_ratio = 0.0
        person_fall_suspected = False

        close_target = False

        for box in r.boxes:
            cls_id = int(box.cls[0])
            name = self.model.names[cls_id]
            conf = float(box.conf[0])

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            box_w = max(0.0, x2 - x1)
            box_h = max(0.0, y2 - y1)
            box_area = box_w * box_h

            if name == "person":
                person_count += 1
                h_ratio = box_h / h
                person_area_ratio = box_area / frame_area
                person_aspect_ratio = box_w / max(box_h, 1.0)

                max_person_h_ratio = max(max_person_h_ratio, h_ratio)
                max_person_aspect_ratio = max(max_person_aspect_ratio, person_aspect_ratio)

                # 人员倒地辅助判断：人体框横向明显大于纵向，且目标面积不能太小
                if person_aspect_ratio >= 1.25 and person_area_ratio >= 0.08:
                    person_fall_suspected = True
                    close_target = True

                if h_ratio >= PERSON_CLOSE_HEIGHT_RATIO:
                    close_target = True

            elif name in OBSTACLE_NAMES:
                obstacle_count += 1
                area_ratio = box_area / frame_area
                max_obstacle_area_ratio = max(max_obstacle_area_ratio, area_ratio)

                if area_ratio >= OBSTACLE_CLOSE_AREA_RATIO:
                    close_target = True

        visual_close_target = close_target

        ultrasonic_hard_stop = (
            self.us_blocked or
            (self.us_distance_cm is not None and self.us_distance_cm < 20.0)
        )

        ultrasonic_confirm_close = (
            self.us_distance_cm is not None and self.us_distance_cm < 30.0
        )

        close_target = ultrasonic_hard_stop or (visual_close_target and ultrasonic_confirm_close)

        return {
            "close_target": close_target,
            "person_count": person_count,
            "obstacle_count": obstacle_count,
            "cargo_count": obstacle_count,
            "max_person_h_ratio": max_person_h_ratio,
            "max_obstacle_area_ratio": max_obstacle_area_ratio,
            "max_person_aspect_ratio": max_person_aspect_ratio,
            "person_fall_suspected": person_fall_suspected,
            "visual_close_target": visual_close_target,
            "ultrasonic_blocked": self.us_blocked,
            "ultrasonic_distance_cm": self.us_distance_cm,
        }

    def classify_event(self, info):
        if info.get("person_fall_suspected", False):
            return "person_fall_warning"

        if info.get("obstacle_count", 0) >= 2 and info.get("max_obstacle_area_ratio", 0.0) >= 0.03:
            return "stacking_abnormal"

        if info.get("max_obstacle_area_ratio", 0.0) >= 0.05:
            return "cargo_blocking"

        if info.get("max_person_h_ratio", 0.0) >= PERSON_CLOSE_HEIGHT_RATIO:
            return "person_near_warning"

        return "near_obstacle"

    def save_event(self, frame, info):
        now = datetime.now()
        ts = now.strftime("%Y%m%d_%H%M%S")

        image_path = EVENT_DIR / f"obstacle_{ts}.jpg"
        cv2.imwrite(str(image_path), frame)

        event_type = self.classify_event(info)

        line = (
            f"{now.strftime('%Y-%m-%d %H:%M:%S')}, "
            f"state=OBSTACLE, "
            f"event_type={event_type}, "
            f"event_type={event_type}, "
            f"person_count={info['person_count']}, "
            f"obstacle_count={info['obstacle_count']}, "
            f"max_person_h_ratio={info['max_person_h_ratio']:.2f}, "
            f"max_person_aspect_ratio={info.get('max_person_aspect_ratio', 0.0):.2f}, "
            f"max_obstacle_area_ratio={info['max_obstacle_area_ratio']:.3f}, "
            f"image={image_path}\n"
        )

        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(line)

        print("[patrol] saved image:", image_path)

    def bypass(self):
        print("[patrol] bypass start")

        actions = [
            ("car_back 70", 0.5),
            ("car_right 70", 0.6),
            ("car_forward 70", 1.0),
            ("car_left 70", 0.6),
            ("car_stop", 0.3),
        ]

        for cmd, delay in actions:
            self.send_cmd(cmd, force=True)
            time.sleep(delay)

        print("[patrol] bypass done, recheck")

    def recheck_after_bypass(self):
        """
        绕行完成后不要立刻判断失败。
        等待摄像头稳定后连续检测 5 帧：
        - 至少 3 帧清空：绕行成功
        - 否则：绕行失败
        """
        print("[patrol] bypass done, wait camera stable")
        time.sleep(1.0)

        clear_count = 0
        blocked_count = 0

        for i in range(5):
            ret, frame = self.cap.read()

            if not ret:
                print(f"[patrol] recheck frame {i + 1}/5 read failed")
                blocked_count += 1
                time.sleep(0.2)
                continue

            self.poll_ultrasonic(force=True)
            info = self.detect_frame(frame)

            if info.get("close_target", False):
                blocked_count += 1
                print(
                    f"[patrol] recheck {i + 1}/5 blocked, "
                    f"person={info.get('person_count', 0)}, "
                    f"obstacle={info.get('obstacle_count', 0)}, "
                    f"person_h={info.get('max_person_h_ratio', 0.0):.2f}, "
                    f"obs_area={info.get('max_obstacle_area_ratio', 0.0):.3f}"
                )
            else:
                clear_count += 1
                print(f"[patrol] recheck {i + 1}/5 clear")

            time.sleep(0.2)

        print(f"[patrol] recheck result: clear={clear_count}, blocked={blocked_count}")

        return clear_count >= 3


    def print_status(self, info, fps):
        now = time.time()
        if now - self.last_print_time >= 1.0:
            print(
                f"state={self.state}, "
                f"person={info['person_count']}, "
                f"cargo={info['obstacle_count']}, "
                f"person_h={info['max_person_h_ratio']:.2f}, "
                f"obs_area={info['max_obstacle_area_ratio']:.3f}, "
                f"fps={fps:.1f}"
            )
            self.last_print_time = now

    def run(self):
        last_time = time.time()

        try:
            while rclpy.ok():
                ret, frame = self.cap.read()
                if not ret:
                    print("[camera] read frame failed")
                    time.sleep(0.2)
                    continue

                now = time.time()
                fps = 1.0 / max(now - last_time, 0.001)
                last_time = now

                self.poll_ultrasonic()
                info = self.detect_frame(frame)
                self.update_video_stream(frame, info)
                self.print_status(info, fps)

                if self.state == "PATROL":
                    if info["close_target"]:
                        self.close_count += 1
                    else:
                        self.close_count = 0

                    if self.close_count >= CLOSE_CONFIRM_FRAMES:
                        self.state = "OBSTACLE"
                    else:
                        self.send_cmd(f"car_forward {SPEED}")

                elif self.state == "OBSTACLE":
                    self.send_cmd("car_stop", force=True)
                    print(f"[patrol] obstacle confirmed ({self.classify_event(info)}) -> stop and save image")
                    self.save_event(frame, info)
                    self.state = "BYPASS"

                elif self.state == "BYPASS":
                    self.bypass()
                    ok = self.recheck_after_bypass()

                    if ok:
                        print("[patrol] bypass success -> continue patrol")
                        self.close_count = 0
                        self.state = "PATROL"
                    else:
                        print("[patrol] bypass failed -> stopped")
                        self.stopped_clear_count = 0
                        self.stopped_printed = False
                        self.state = "STOPPED"

                elif self.state == "STOPPED":
                    self.send_cmd("car_stop")

                    if not self.stopped_printed:
                        print("[patrol] stopped, need manual handling")
                        self.stopped_printed = True

                    if info["close_target"]:
                        self.stopped_clear_count = 0
                    else:
                        self.stopped_clear_count += 1

                    if self.stopped_clear_count >= STOPPED_CLEAR_FRAMES:
                        print("[patrol] area clear -> resume patrol")
                        self.close_count = 0
                        self.stopped_clear_count = 0
                        self.stopped_printed = False
                        self.state = "PATROL"

                    time.sleep(0.3)

        except KeyboardInterrupt:
            print("[patrol] Ctrl+C stop")

        finally:
            self.send_cmd("car_stop", force=True)
            self.cap.release()
            self.ser.close()
            print("[patrol] exit")


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = PatrolNode()
        node.run()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
