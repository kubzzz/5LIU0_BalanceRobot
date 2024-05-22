#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuSubscriberNode(Node):

    def __init__(self):
        super().__init__("imu_subscriber")
        self.imu_subscriber = self.create_subscription(Imu, "/imu_plugin/out", self.imu_callback, 10)

    def imu_callback(self, msg: Imu):
        theta_prev = msg.angular_velocity.x
        a_y = msg.linear_acceleration.y
        a_z = msg.linear_acceleration.z
        phi = math.atan2(a_y, a_z)*57.295780

        self.get_logger().info(str(phi))

def main(args=None):
    rclpy.init(args=args)

    node = ImuSubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()