import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200


class KeyboardCarNode(Node):
    def __init__(self):
        super().__init__("keyboard_car_node")
        self.speed = 30
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)

        self.get_logger().info(f"PSoC62 serial opened: {PORT}")
        self.get_logger().info("w=forward, s=back, a=left, d=right, space/x=stop")
        self.get_logger().info("1=30%, 2=40%, 3=50%, 4=60%, 5=70%, q=quit")

    def send_cmd(self, cmd):
        self.get_logger().info("send: " + cmd)
        self.ser.write((cmd + "\r\n").encode())

    def stop(self):
        self.send_cmd("car_stop")

    def handle_key(self, key):
        if key == "w":
            self.send_cmd(f"car_forward {self.speed}")
        elif key == "s":
            self.send_cmd(f"car_back {self.speed}")
        elif key == "a":
            self.send_cmd(f"car_left {self.speed}")
        elif key == "d":
            self.send_cmd(f"car_right {self.speed}")
        elif key == " " or key == "x":
            self.stop()
        elif key == "1":
            self.speed = 30
            self.get_logger().info("speed = 30%")
        elif key == "2":
            self.speed = 40
            self.get_logger().info("speed = 40%")
        elif key == "3":
            self.speed = 50
            self.get_logger().info("speed = 50%")
        elif key == "4":
            self.speed = 60
            self.get_logger().info("speed = 60%")
        elif key == "5":
            self.speed = 70
            self.get_logger().info("speed = 70%")


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCarNode()

    try:
        node.stop()

        while rclpy.ok():
            key = get_key()

            if key == "q":
                node.stop()
                break

            node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        node.stop()

    finally:
        node.stop()
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
