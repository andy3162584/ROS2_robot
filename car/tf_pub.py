import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

#-----------------------------------------------------------------------------#

class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')

        # --- 廣播 : 發送資料到ROS2中 ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- 訂閱者 : 接收odom ---
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        # 建立 TF 訊息
        t = TransformStamped()
        
        # 保持時間戳與數據絕對同步
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # 直接從 odom 話題複製座標與旋轉
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # 廣播出去
        self.tf_broadcaster.sendTransform(t)

#-----------------------------------------------------------------------------#

def main(args=None):            # ROS2 節點main函式
    rclpy.init(args=args)       # ROS2 節點主入口main初始化
    node = OdomToTFNode()      # 建立ROS2 節點物件並進行初始化
    try:
        rclpy.spin(node)        # 循環等待ROS2退出
    except KeyboardInterrupt:   # 等到按下ctrl+c才會停止
        pass
    finally:
        node.destroy_node()     # 銷毀節點實例
        rclpy.shutdown()        # 關閉ROS2 Python介面

if __name__ == '__main__':
    main()