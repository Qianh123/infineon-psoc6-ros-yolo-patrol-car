import os
import time
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
SPEED = 30

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


class PatrolNode(Node):
    def __init__(self):
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
                    print(line)
        except Exception:
            pass

        if force or not cmd.startswith("car_forward"):
            print("[car] send", cmd)

        self.last_cmd = cmd
        self.last_cmd_time = now

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

        return {
            "close_target": close_target,
            "person_count": person_count,
            "obstacle_count": obstacle_count,
            "max_person_h_ratio": max_person_h_ratio,
            "max_obstacle_area_ratio": max_obstacle_area_ratio,
            "max_person_aspect_ratio": max_person_aspect_ratio,
            "person_fall_suspected": person_fall_suspected,
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
            ("car_back 30", 0.5),
            ("car_right 30", 0.6),
            ("car_forward 30", 1.0),
            ("car_left 30", 0.6),
            ("car_stop", 0.3),
        ]

        for cmd, delay in actions:
            self.send_cmd(cmd, force=True)
            time.sleep(delay)

        print("[patrol] bypass done, recheck")

    def recheck_after_bypass(self):
        close_frames = 0

        for _ in range(10):
            ret, frame = self.cap.read()
            if not ret:
                continue

            info = self.detect_frame(frame)

            if info["close_target"]:
                close_frames += 1

            time.sleep(0.1)

        return close_frames < 3

    def print_status(self, info, fps):
        now = time.time()
        if now - self.last_print_time >= 1.0:
            print(
                f"state={self.state}, "
                f"person={info['person_count']}, "
                f"obstacle={info['obstacle_count']}, "
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

                info = self.detect_frame(frame)
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
