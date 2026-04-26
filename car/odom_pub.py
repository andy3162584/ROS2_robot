import rclpy                    # ROS2 Python介面函式庫
from rclpy.node import Node     # 節點類別
from std_msgs.msg import Int32MultiArray    # 匯入訊息類型(傳輸一串整數)
from nav_msgs.msg import Odometry           # 匯入訊息類型(定位格式)
from sensor_msgs.msg import Imu             # 匯入訊息類型(傳送IMU資料格式)
from geometry_msgs.msg import TransformStamped  # 匯入訊息類型(TF資料格式)
import math

#-----------------------------------------------------------------------------#

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # --- 發送者 : 發送資料到ROS2中 ---
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # --- 訂閱者 : 接收motor/imu ---
        self.create_subscription(Int32MultiArray, 'motor_pos', self.listener_callback, 10)
        self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        
        # --- 實體參數調整 ---
        self.wheel_radius = 0.035  
        self.ticks_per_rev = 990.0
        self.meter_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.right_rear_fix = 0.934 # 右後輪修正係數
        
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_ticks = None
        self.gyro_z = 0.0
        self.last_imu_time = None
        
        # IMU 校準狀態
        self.calibrated = False
        self.cali_count = 0

        # 定時器：50Hz 更新 TF
        self.timer = self.create_timer(0.02, self.odom_pub)
        self.get_logger().info('!!! ODOM V3.1: FORCING CLOCK SYNC !!!')

    def imu_callback(self, msg):
        # 核心修正：直接使用節點現在的時間，無視 IMU 驅動可能帶有的錯誤時戳
        now_time = self.get_clock().now()
        
        self.gyro_z = -msg.angular_velocity.z # 修正左轉為正

        if not self.calibrated:
            self.cali_count += 1
            if self.cali_count >= 50:
                self.calibrated = True
                self.get_logger().info('IMU Calibration Done.')
            return

        if self.last_imu_time is not None:
            # 使用節點時鐘計算 dt
            dt = (now_time - self.last_imu_time).nanoseconds / 1e9
            if dt > 0:
                self.th += self.gyro_z * dt
                self.th = math.atan2(math.sin(self.th), math.cos(self.th))
                
        self.last_imu_time = now_time

    def listener_callback(self, msg):
        # 只要 IMU 資料還沒進來，就不更新座標，避免跳變
        if len(msg.data) < 4 or self.last_imu_time is None:
            return
        
        # 右後輪數據修正
        r_rear_fixed = msg.data[3] * self.right_rear_fix
        curr_left = (msg.data[0] + msg.data[1]) / 2.0
        curr_right = (msg.data[2] + r_rear_fixed) / 2.0
        
        if self.last_ticks is not None:
            d_left = (curr_left - self.last_ticks[0]) * self.meter_per_tick
            d_right = (curr_right - self.last_ticks[1]) * self.meter_per_tick
            dist = (d_left + d_right) / 2.0
            
            # 直走保護：轉向極小時強制直線前進
            self.x += dist * math.cos(self.th)
            self.y += dist * math.sin(self.th)

        self.last_ticks = [curr_left, curr_right]

    def odom_pub(self):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.th / 2.0); qw = math.cos(self.th / 2.0)

        # 發布 Odometry 訊息
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_publisher.publish(odom)

#-----------------------------------------------------------------------------#

def main(args=None):            # ROS2 節點main函式
    rclpy.init(args=args)       # ROS2 節點主入口main初始化
    node = OdomPublisher()      # 建立ROS2 節點物件並進行初始化
    try:
        rclpy.spin(node)        # 循環等待ROS2退出
    except KeyboardInterrupt:   # 等到按下ctrl+c才會停止
        pass
    finally:
        node.destroy_node()     # 銷毀節點實例
        rclpy.shutdown()        # 關閉ROS2 Python介面

if __name__ == '__main__':
    main()