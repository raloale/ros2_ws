#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriberNode(Node):

    def __init__(self):
        super().__init__("camera_sensor_subcriber")
        self.get_logger().info("CameraSubscriberNode init")
        self.subcriber_ = self.create_subscription(Image, 'video_topic', self.frame_received, 60)
        self.get_logger().info("CameraSubscriberNode Started")
        self.bridge = CvBridge()

    def frame_received(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        frame_id = msg.header.frame_id
        cv2.putText(frame, "Frame Id: %s"%frame_id, (30,30), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()