import os
from ament_index_python.packages import get_package_share_directory     # 查詢功能套件路徑的方法
from launch import LaunchDescription            # launch開機檔案的描述類別
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node             # 啟動節點的描述類型

def generate_launch_description():      # 自動生成launch開機檔案的函式
    pkg_path = get_package_share_directory('car')

    # 1. 啟動機器人模型 (URDF/RSP)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'rsp.launch.py')])
    )

    # 2. 啟動雷達 (RPLIDAR)
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ])
    )

    # 3. 硬體節點
    esp32_bridge = Node(
        package='car',      # 節點所在的功能套件
        executable='esp32_bridge',  # 節點的可執行檔案
        output='screen'
    )
    
    # 4. 確保 odom_pub 啟動時會抓到你最新編譯的 V3.0
    odom_pub = Node(
        package='car', 
        executable='odom_pub', 
        output='screen',
        parameters=[{'use_sim_time': False}] 
    )

    # odom -> base_link的廣播交由EKF處理
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'ekf.yaml')]
    )

    # tf_pub = Node(
    #     package='car', 
    #     executable='tf_pub', 
    #     output='screen',
    #     parameters=[{'use_sim_time': False}] 
    # )


    return LaunchDescription([
        rsp, 
        lidar, 
        esp32_bridge, 
        odom_pub,
        ekf_node
    ])