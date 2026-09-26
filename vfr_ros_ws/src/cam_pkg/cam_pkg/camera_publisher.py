#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__("camera_sensor_publisher")
        self.get_logger().info("CameraPublisherNode init")
        self.publisher_ = self.create_publisher(Image, "video_topic", 1)
        self.frameId = 0

        self.videoCapture = cv2.VideoCapture('rtsp://10.22.59.5/jpegsen2src1n2')
        self.bridge = CvBridge()

        # Modo 1 - timer + spin
        self.create_timer(1/30, self.timer_callback)
        self.get_logger().info("CameraPublisherNode Started")

    def timer_callback(self):
        self.frameId += 1
        r, frame = self.videoCapture.read()
        imageMsg = self.bridge.cv2_to_imgmsg(frame)
        imageMsg.header.frame_id = "map"
        self.publisher_.publish(imageMsg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()