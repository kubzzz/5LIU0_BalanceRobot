import rclpy
from rclpy import Node
from sensor_msgs.msg import Imu

class ImuSubscriberNode(Node):

    def __init__(self):
        super().__init__("imu_subscriber")
        self.imu_subscriber = self.create_subscription(Imu, "/imu_plugin/out", self.imu_callback, 10)

    def imu_callback(self, msg: Imu):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)

    node = ImuSubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()