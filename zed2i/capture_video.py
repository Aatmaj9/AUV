#!/usr/bin/env python3
import sys, os, termios, select, time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

TOPIC = "/zed/zed_node/left/image_rect_color"
DURATION_SEC = 10.0
ASSUMED_FPS = 30.0  # metadata only

def enable_noecho():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~(termios.ECHO | termios.ICANON)
    new[6][termios.VMIN] = 0
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

class VideoSaver(Node):
    def __init__(self, save_dir):
        super().__init__("zed_video_saver")
        self.bridge = CvBridge()
        self.save_dir = Path(save_dir); self.save_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, TOPIC, self.cb, qos_profile_sensor_data)
        self.recording = False
        self.start_t = 0.0
        self.writer = None
        print("Press 'V' to record , Ctrl+C to exit")

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.recording:
            if self.writer is None:
                h, w = frame.shape[:2]
                name = datetime.now().strftime("%Y%m%d_%H%M%S") + ".avi"
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                self.path = str(self.save_dir / name)
                self.writer = cv2.VideoWriter(self.path, fourcc, ASSUMED_FPS, (w, h))
                self.start_t = time.monotonic()
                print("Capture started, wait 10 secs")

            self.writer.write(frame)

            if time.monotonic() - self.start_t >= DURATION_SEC:
                self.writer.release()
                self.writer = None
                self.recording = False
                print("video captured")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            k = get_key()
            if k in ('v', 'V'):
                if not self.recording:
                    self.recording = True
            elif k in ('\x03', '\x04'):
                break

def main():
    if len(sys.argv) < 2: sys.exit(1)
    rclpy.init()
    node = VideoSaver(sys.argv[1])
    fd = None; old = None
    try:
        fd, old = enable_noecho()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.writer is not None:
            node.writer.release()
        if fd is not None and old is not None:
            restore_tty(fd, old)
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
