import sys
import time
import select
import termios
import tty

import rclpy
from rclpy.node import Node

import serial


PORT = "/dev/ttyACM0"
BAUD = 115200

DEFAULT_SPEED = 100

# 松开按键后多久自动停车，越小越灵敏
HOLD_TIMEOUT = 0.25
LOOP_INTERVAL = 0.02

SPEED_LEVELS = {
    "1": 30,
    "2": 50,
    "3": 70,
    "4": 85,
    "5": 100,
}


class KeyboardCarNode(Node):
    def __init__(self):
        super().__init__("keyboard_car_node")

        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(1.0)

        self.speed = DEFAULT_SPEED
        self.last_cmd = None
        self.last_key_time = 0.0
        self.running = True

        self.get_logger().info("keyboard hold-control with speed started")
        self.print_help()

        self.send_cmd("car_stop", force=True)

    def print_help(self):
        print("")
        print("========== ROS2 Keyboard Hold Control ==========")
        print("按住 w：前进；松开：停止")
        print("按住 s：后退；松开：停止")
        print("按住 a：左转；松开：停止")
        print("按住 d：右转；松开：停止")
        print("")
        print("调速：")
        print("1 = 30")
        print("2 = 50")
        print("3 = 70")
        print("4 = 85")
        print("5 = 100")
        print("")
        print("空格 / x：立即停止")
        print("q：退出")
        print("当前默认速度：", self.speed)
        print("===============================================")
        print("")

    def build_move_cmd(self, key):
        if key == "w":
            return f"car_forward {self.speed}"
        if key == "s":
            return f"car_back {self.speed}"
        if key == "a":
            return f"car_left {self.speed}"
        if key == "d":
            return f"car_right {self.speed}"
        return None

    def send_cmd(self, cmd, force=False):
        if (not force) and cmd == self.last_cmd:
            return

        self.ser.write((cmd + "\r\n").encode())
        time.sleep(0.03)

        out = self.ser.read_all().decode(errors="ignore")
        if out.strip():
            print(out.strip())

        print("[car] send", cmd)
        self.last_cmd = cmd

    def set_speed(self, key):
        self.speed = SPEED_LEVELS[key]
        print(f"[speed] set speed = {self.speed}")

    def run_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok() and self.running:
                now = time.time()

                readable, _, _ = select.select([sys.stdin], [], [], LOOP_INTERVAL)

                if readable:
                    ch = sys.stdin.read(1).lower()
                    now = time.time()

                    if ch in SPEED_LEVELS:
                        self.set_speed(ch)

                    elif ch in ["w", "s", "a", "d"]:
                        cmd = self.build_move_cmd(ch)
                        self.last_key_time = now
                        self.send_cmd(cmd)

                    elif ch == " " or ch == "x":
                        self.send_cmd("car_stop", force=True)
                        self.last_key_time = 0.0

                    elif ch == "q":
                        self.send_cmd("car_stop", force=True)
                        self.running = False
                        break

                # 松开方向键后，超过 HOLD_TIMEOUT 没有继续收到按键，就自动停车
                if self.last_cmd and self.last_cmd != "car_stop":
                    if now - self.last_key_time > HOLD_TIMEOUT:
                        self.send_cmd("car_stop", force=True)

        except KeyboardInterrupt:
            pass

        finally:
            self.send_cmd("car_stop", force=True)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.ser.close()
            print("[keyboard] exit")


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardCarNode()

    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
