import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFixer(Node):
    def __init__(self):
        super().__init__('scan_fixer')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.publisher_ = self.create_publisher(LaserScan, '/scan_fixed', 10)

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame' # 確保與 URDF 一致
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScanFixer())
    rclpy.shutdown()