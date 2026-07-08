import os
import time
import json
from statistics import median
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
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
LATEST_STATUS = {
    "server": "running",
    "person": 0,
    "cargo": 0,
    "distance": None,
    "state": "INIT",
    "reason": "init",
    "close": False,
    "slow_down": False,
    "emergency_stop": False
}
API_ACTION = None
JPEG_LOCK = threading.Lock()



NORMAL_SPEED = 70
SLOW_SPEED = 40
BYPASS_SPEED = 70
HARD_STOP_CM = 15.0
FUSION_STOP_CM = 30.0
SLOW_DOWN_CM = 50.0

class VideoStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_POST(self):
        self.do_GET()

    def do_GET(self):
        global LATEST_JPEG, LATEST_STATUS, API_ACTION

        if self.path.startswith("/api/status"):
            data = dict(LATEST_STATUS)
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/api/stop"):
            API_ACTION = "stop"
            LATEST_STATUS["emergency_stop"] = True
            LATEST_STATUS["state"] = "STOPPED"
            LATEST_STATUS["reason"] = "api_stop"

            body = json.dumps({"ok": True, "action": "stop"}, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/api/resume"):
            API_ACTION = "resume"
            LATEST_STATUS["emergency_stop"] = False
            LATEST_STATUS["reason"] = "api_resume"

            body = json.dumps({"ok": True, "action": "resume"}, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return

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
    server = ThreadingHTTPServer(("0.0.0.0", 8090), VideoStreamHandler)
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
        self.us_filtered_distance_cm = None
        self.us_distance_history = []
        self.decision_reason = "init"
        self.last_us_poll_time = 0.0
        self.last_forward_time = 0.0

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
        """解析 PSoC62 超声波输出。"""
        try:
            if not line:
                return

            # blocked=0 / blocked=1
            m = re.search(r"blocked=([0-9]+)", line)
            if m:
                self.us_blocked = (m.group(1) == "1")

            # [us_guard] obstacle 12.95 cm -> blocked and car_stop
            # [us_guard] clear 103.12 cm -> unblock
            m = re.search(r"(obstacle|clear) +([0-9.]+) *cm", line)
            if m:
                kind = m.group(1)
                dist = float(m.group(2))
                self.us_distance_cm = dist
                self.us_blocked = (kind == "obstacle")
                self.us_distance_history = [dist]
                self.us_filtered_distance_cm = dist
                return

            # [us_guard] running=1, blocked=0, last_distance=98.10 cm
            m = re.search(r"last_distance=([0-9.]+) *cm", line)
            if m:
                dist = float(m.group(1))
                self.us_distance_cm = dist
                self.update_distance_filter(dist)
                return

            if "last_distance=invalid" in line:
                self.us_distance_cm = None
                return

            # [us] ... distance=9.55 cm
            m = re.search(r"distance=([0-9.]+) *cm", line)
            if m:
                dist = float(m.group(1))
                self.us_distance_cm = dist
                self.update_distance_filter(dist)
                return

            # [safe] forward blocked by ultrasonic 15.97 cm
            m = re.search(r"ultrasonic +([0-9.]+) *cm", line)
            if m:
                dist = float(m.group(1))
                self.us_distance_cm = dist
                self.us_blocked = True
                self.us_distance_history = [dist]
                self.us_filtered_distance_cm = dist
                return

        except Exception as e:
            print("[us] parse failed:", e)

    def update_distance_filter(self, value):
        try:
            if value is None:
                return
            self.us_distance_history.append(float(value))
            self.us_distance_history = self.us_distance_history[-5:]
            self.us_filtered_distance_cm = float(median(self.us_distance_history))
        except Exception:
            pass

    def poll_ultrasonic(self, force=False):
        """
        主动查询超声波距离。
        当前 PSoC62 已验证 us_test 能输出：
        [us] trig=D6/P11_4 echo=D7/P11_5 echo_us=xxx, distance=xx.xx cm
        所以 ROS2 这里直接使用 us_test 获取距离。
        """
        now = time.time()
        if (not force) and (now - self.last_us_poll_time) < 0.35:
            return

        self.last_us_poll_time = now

        try:
            self.ser.write(b"us_test\r\n")
            time.sleep(0.08)
            out = self.ser.read_all().decode(errors="ignore")

            for line in out.splitlines():
                line = line.strip()
                if not line:
                    continue

                if line.startswith("[us]") or line.startswith("[us_guard]") or line.startswith("[safe]"):
                    self.update_us_from_line(line)
                    print(line)

        except Exception as e:
            print("[us] poll failed:", e)


    def update_video_stream(self, frame, info=None):
        global LATEST_JPEG, LATEST_STATUS

        try:
            show = frame.copy()

            if info is not None:
                person_count = info.get("person_count", 0)
                cargo_count = info.get("cargo_count", info.get("obstacle_count", 0))
                distance = info.get("ultrasonic_filtered_cm", self.us_filtered_distance_cm)
                reason = info.get("reason", self.decision_reason)
                close_target = bool(info.get("close_target", False))
                slow_down = bool(info.get("slow_down", False))

                LATEST_STATUS = {
                    "server": "running",
                    "person": int(person_count),
                    "cargo": int(cargo_count),
                    "distance": None if distance is None else float(distance),
                    "state": str(self.state),
                    "reason": str(reason),
                    "close": close_target,
                    "slow_down": slow_down,
                    "emergency_stop": bool(globals().get("API_ACTION") == "stop")
                }

                cv2.rectangle(show, (10, 10), (460, 155), (0, 0, 0), -1)

                cv2.putText(show, f"person: {person_count}", (20, 38),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

                cv2.putText(show, f"cargo: {cargo_count}", (20, 68),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

                if distance is not None:
                    dist_text = f"distance: {distance:.1f} cm"
                else:
                    dist_text = "distance: invalid"

                cv2.putText(show, dist_text, (20, 98),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

                cv2.putText(show, f"state: {self.state}", (20, 125),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                cv2.putText(show, f"reason: {reason}", (20, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 0), 1)

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
        cargo_count = obstacle_count
        vision_target = (person_count > 0 or cargo_count > 0)

        distance = self.us_filtered_distance_cm
        close_target = False
        slow_down = False
        reason = "clear"

        if self.us_blocked or (distance is not None and distance <= HARD_STOP_CM):
            close_target = True
            reason = "ultrasonic_hard_stop"
        elif vision_target and distance is not None and distance <= FUSION_STOP_CM:
            close_target = True
            reason = "vision_ultrasonic_fusion"
        elif vision_target and distance is None:
            close_target = False
            reason = "vision_only_no_distance"
        elif vision_target and distance > FUSION_STOP_CM:
            close_target = False
            reason = "target_far_continue"
        elif distance is not None and FUSION_STOP_CM < distance <= SLOW_DOWN_CM:
            close_target = False
            slow_down = True
            reason = "slow_zone"
        else:
            close_target = False
            reason = "clear"

        self.decision_reason = reason

        return {
            "close_target": close_target,
            "slow_down": slow_down,
            "reason": reason,
            "ultrasonic_filtered_cm": self.us_filtered_distance_cm,
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
        return info.get("reason", self.decision_reason)

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
            (f"car_back {BYPASS_SPEED}", 0.5),
            (f"car_right {BYPASS_SPEED}", 0.6),
            (f"car_forward {BYPASS_SPEED}", 1.0),
            (f"car_left {BYPASS_SPEED}", 0.6),
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
        if now - self.last_print_time < 1.0:
            return
        self.last_print_time = now

        person = info.get("person_count", 0)
        cargo = info.get("cargo_count", info.get("obstacle_count", 0))
        dist = info.get("ultrasonic_distance_cm", self.us_distance_cm)
        filt = info.get("ultrasonic_filtered_cm", self.us_filtered_distance_cm)
        close = 1 if info.get("close_target", False) else 0
        reason = info.get("reason", self.decision_reason)

        dist_s = "invalid" if dist is None else f"{dist:.1f}cm"
        filt_s = "invalid" if filt is None else f"{filt:.1f}cm"

        print(
            f"state={self.state}, person={person}, cargo={cargo}, "
            f"distance={dist_s}, filtered={filt_s}, close={close}, "
            f"reason={reason}, fps={fps:.1f}"
        )

    def handle_api_action(self):
        global API_ACTION

        action = API_ACTION
        if action is None:
            return

        API_ACTION = None

        if action == "stop":
            self.send_cmd("car_stop", force=True)
            self.state = "STOPPED"
            self.decision_reason = "api_stop"
            print("[api] emergency stop")

        elif action == "resume":
            self.close_count = 0
            self.stopped_clear_count = 0
            self.state = "PATROL"
            self.decision_reason = "api_resume"
            print("[api] resume patrol")

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
                self.handle_api_action()

                if self.state == "PATROL":
                    if info.get("close_target", False):
                        self.close_count += 1
                    else:
                        self.close_count = 0

                    if self.close_count >= CLOSE_CONFIRM_FRAMES:
                        self.send_cmd("car_stop", force=True)
                        self.state = "OBSTACLE"
                    else:
                        patrol_speed = SLOW_SPEED if info.get("slow_down", False) else NORMAL_SPEED

                        # 关键修复：
                        # 只要处于 PATROL 且 close=0，就周期性强制发送前进命令。
                        # 避免前面被 car_stop 后，状态恢复了但底层没有重新收到 forward。
                        now_cmd = time.time()
                        if now_cmd - self.last_forward_time >= 0.5:
                            self.send_cmd(f"car_forward {patrol_speed}", force=True)
                            self.last_forward_time = now_cmd

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
