import rclpy                            # ROS2 Python介面函式庫
from rclpy.node import Node             # 節點類別
from std_msgs.msg import Int32MultiArray    # 匯入訊息類型(傳輸一串整數)
from sensor_msgs.msg import Imu             # 匯入訊息類型(傳送IMU資料格式)
from geometry_msgs.msg import Twist         # 匯入訊息類型(標準的速度指令)
import serial
import math

#-----------------------------------------------------------------------------#

class esp32bridge(Node):
    def __init__(self,name):
        super().__init__(name)        # ROS2節點父類別初始化，並建立節點
        # 已確認路徑為 /dev/ttyAMA0
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
        
        # --- 發送者 : 發送資料到ROS2中 ---
        self.pos_pub = self.create_publisher(Int32MultiArray, 'motor_pos', 10)      #建立發行者物件(訊息類型, 話題名稱, 序列長度)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        
        # --- 訂閱者 : 接收鍵盤或導航發出的指令 ---
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)    #建立訂閱者物件(訊息類型, Topic的名字, 訂閱者回呼函式, 序列長度)
        
        self.timer = self.create_timer(0.02, self.update)               # 建立計時器，定時呼叫update函式執行
        self.get_logger().info("Motor Bridge Started, listening to /cmd_vel...")    # ROS2日誌輸出

    # --- 接收後處理訊息並傳給ESP32 ---
    def cmd_vel_callback(self, msg):
        v = msg.linear.x        # 線速度 m/s
        w = msg.angular.z       # 角速度 rad/s

        scale = 520.0  # 統一的映射係數
        track_width = 0.34 # 實測兩輪間距

        # 使用標準差速公式
        # v_left = v - w * L / 2
        # v_right = v + w * L / 2
        left_speed = int((v - (w * track_width / 2.0)) * scale)
        right_speed = int((v + (w * track_width / 2.0)) * scale)

        # 限制範圍並發送
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        cmd = f"{left_speed},{right_speed}\n"
        self.ser.write(cmd.encode('utf-8'))     # 寫回給ESP32

    # --- 更新ESP32回傳的資料並發布 ---
    def update(self):
        if self.ser.in_waiting > 0:         # 確認raspberry裡的緩衝區有資料
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()     #從ESP32讀取資料
                data = line.split(',')  # 根據逗號分割字串(posLF, posLB, -posRF, -posRB, cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz)

                if len(data) < 10:      # 如果資料長度不對
                    return              # 中斷後續執行

                # 1. 發布編碼器數據
                ticks_msg = Int32MultiArray()
                ticks_msg.data = [int(data[0]), int(data[1]), int(data[2]), int(data[3])]
                self.pos_pub.publish(ticks_msg)      # 發布話題訊息
                
                # 2. 發布 IMU 數據
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"
                
                # 加速度
                imu_msg.linear_acceleration.x = float(data[4])
                imu_msg.linear_acceleration.y = float(data[5])
                imu_msg.linear_acceleration.z = float(data[6]) 
                
                # 角速度 (陀螺儀)
                imu_msg.angular_velocity.x = float(data[7])
                imu_msg.angular_velocity.y = float(data[8])
                
                # 關鍵：確認 data[9] 真的有數值
                # 加上原本的方向與單位修正
                raw_gz = float(data[9]) * (math.pi / 180.0) * -1.0
                imu_msg.angular_velocity.z = raw_gz

                # 填充協方差 (維持原樣)
                imu_msg.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
                imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
                imu_msg.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
                
                # 強制賦值 orientation 的 W 軸，避免四元數無效
                imu_msg.orientation.w = 1.0 
                
                self.imu_pub.publish(imu_msg)       # 發布話題訊息
                
            except Exception as e:
                self.get_logger().error(f'Error parsing serial data: {e}')      # 如果出現錯誤，輸出error警示

#-----------------------------------------------------------------------------#

def main(args=None):            # ROS2 節點main函式
    rclpy.init(args=args)       # ROS2 節點主入口main初始化
    node = esp32bridge('esp32_bridge')    # 建立ROS2 節點物件並進行初始化，命名節點名稱為motor_bridge
    try:
        rclpy.spin(node)        # 循環等待ROS2退出
    except KeyboardInterrupt:   # 等到按下ctrl+c才會停止
        pass
    finally:
        node.ser.close()
        node.destroy_node()     # 銷毀節點實例
        rclpy.shutdown()        # 關閉ROS2 Python介面

if __name__ == '__main__':
    main()