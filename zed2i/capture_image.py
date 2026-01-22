#!/usr/bin/env python3
import sys, os, termios, select
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

TOPIC = "/zed/zed_node/left/image_rect_color"  # or /zed/zed_node/rgb/image_rect_color

def enable_noecho():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~(termios.ECHO | termios.ICANON)  # lflags: no echo, non-canonical
    new[6][termios.VMIN] = 0                    # read returns immediately
    new[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, new)
    return fd, old

def restore_tty(fd, old):
    termios.tcsetattr(fd, termios.TCSANOW, old)

def get_key(timeout=0.05):
    if not sys.stdin.isatty():
        return None
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if r:
        return os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
    return None

class ImageSaver(Node):
    def __init__(self, save_dir):
        super().__init__("zed_image_saver")
        self.bridge = CvBridge()
        self.latest = None
        self.save_dir = Path(save_dir); self.save_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, TOPIC, self.cb, qos_profile_sensor_data)
        print("Press 'C' to capture, Ctrl+C to exit")

    def cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            k = get_key()
            if k == 'c' and self.latest is not None:
                name = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".png"
                cv2.imwrite(str(self.save_dir / name), self.latest)
                print("image captured")
            elif k in ('\x03', '\x04'):
                break

def main():
    if len(sys.argv) < 2: sys.exit(1)
    rclpy.init()
    node = ImageSaver(sys.argv[1])
    fd = None; old = None
    try:
        fd, old = enable_noecho()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if fd is not None and old is not None:
            restore_tty(fd, old)
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
