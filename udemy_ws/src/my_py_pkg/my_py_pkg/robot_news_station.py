#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station")
        self.get_logger().info("Hello RobotNewsStation")
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.counter_ = 0
        self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Hello RobotNewsStation Started")

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello ROS22 %s" % self.counter_)
        self.publish_news("Hello ROS22 %s" % self.counter_)

    def publish_news(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()