#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LiveStreamNode(Node):
    def __init__(self):
        super().__init__('livestream')

        # QoS compatible with RViz2 Image display (RELIABLE)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Use a standard camera topic name
        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image_raw',
            qos_profile
        )

        # Open camera 0
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open USB camera.')
            raise RuntimeError('Camera not detected')

        # Optional: set resolution and FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        #self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.bridge = CvBridge()

        # Run at ~30 Hz
        self.timer = self.create_timer(1.0 / 5.0, self.timer_callback)

        self.get_logger().info('LiveStreamNode started, publishing on /camera/image_raw')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from camera.')
            return

        # ---- FIX MIRROR IMAGE: flip horizontally ----
        frame = cv2.flip(frame, 1)   # 1 => left-right flip

        # Convert OpenCV image to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        self.publisher_.publish(msg)

    def destroy_node(self):
        # Release camera on shutdown
        if self.cap.isOpened():
            self.cap.release()
        self.get_logger().info('LiveStreamNode shutting down, camera released.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiveStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
