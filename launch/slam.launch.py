import os
from ament_index_python.packages import get_package_share_directory     # 查詢功能套件路徑的方法
from launch import LaunchDescription            # launch開機檔案的描述類別
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node         # 啟動節點的描述類型

def generate_launch_description():      # 自動生成launch開機檔案的函式
    pkg_path = get_package_share_directory('car')

    # 1. 啟動機器人節點
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'hardware.launch.py')])
    )

    # 2. 這裡確認帶入我們剛剛改好的 yaml 路徑
    slam_params_path = os.path.join(pkg_path, 'config', 'slam_params.yaml')
    
    slam = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
                ]),
                launch_arguments={
                    'slam_params_file': slam_params_path,
                    'use_sim_time': 'False'
                }.items()
            )
        ]
    )

    # 3. 啟動 Rviz2
    rviz = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2', 
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'slam.rviz')],
        output='screen'
    )

    return LaunchDescription([
        hardware,
        slam,
        rviz
    ])