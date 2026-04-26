import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('car')

    # 1. 啟動基礎硬體 (RSP, Lidar, Bridge, Odom)
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'hardware.launch.py')])
    )

    # 2. 定義地圖檔案路徑
    map_file_path = os.path.join(pkg_path, 'maps', 'map.yaml')
    
    # 3. 定義 Nav2 參數檔路徑 (通常會從 nav2 套件複製一份出來修改)
    nav2_params_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    # 4. 啟動 Nav2 Bringup (包含 Map Server, AMCL 定位, 以及 Navigation 堆疊)
    nav2 = TimerAction(
        period=3.0,  # 延遲啟動
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                ]),
                launch_arguments={
                    'map': map_file_path,
                    'params_file': nav2_params_path,
                    'use_sim_time': 'False',
                }.items()
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'nav2.rviz')],
        output='screen'
    )

    return LaunchDescription([
        hardware,
        nav2,
        rviz
    ])