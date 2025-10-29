

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

class UVOverlayNode(Node):
    def __init__(self):
        super().__init__('uv_overlay_node')

        self.bridge = CvBridge()

        # Subscribers via message_filters for time sync
        self.image_sub = Subscriber(self, Image, "/zed2i/left/image_rect_color")
        self.uv_sub    = Subscriber(self, PoseArray, "/uv_xyz")

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.uv_sub],
            queue_size=10,
            slop=0.1   # seconds of allowed timestamp difference
        )
        self.ts.registerCallback(self.synced_callback)

    def synced_callback(self, image_msg, uv_msg):
        # Convert ROS image -> OpenCV (BGR)
        try:
            cam_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")
            return

        h, w = cam_image.shape[:2]

        # Build an empty overlay of the same size as the image
        uv_overlay = np.zeros((h, w, 3), dtype=np.uint8)

        # Draw UV points (stored in orientation.x / orientation.y as per your current message)
        if uv_msg.poses:
            # Extract and validate UV coordinates against current image size
            uvs = np.array([[int(p.orientation.x), int(p.orientation.y)] for p in uv_msg.poses], dtype=int)
            valid = (uvs[:, 0] >= 0) & (uvs[:, 0] < w) & (uvs[:, 1] >= 0) & (uvs[:, 1] < h)
            uvs = uvs[valid]

            radius = 3
            for x, y in uvs:
                # Red dots for UV points
                cv2.circle(uv_overlay, (x, y), radius, (0, 0, 255), -1)

        # Blend overlay on top of camera image
        # alpha controls the visibility of UV points “glow” against the image
        alpha = 0.5
        blended = cv2.addWeighted(cam_image, 1.0, uv_overlay, alpha, 0.0)

        # Optional: draw small crosshairs or labels (uncomment if you want)
        # for x, y in uvs:
        #     cv2.drawMarker(blended, (x, y), (255, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=8, thickness=1)
        #     cv2.putText(blended, f"({x},{y})", (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)

        # Show result
        cv2.imshow("UV Overlay on Camera", blended)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = UVOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
