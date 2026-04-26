# 匯入setuptools的 find_packages 和 setup 函式
from setuptools import find_packages,setup
import os
from glob import glob

# 定義套件名稱
package_name = 'car'

# 定義套件的安裝資訊
setup(
    # 套件名稱
    name=package_name,
    # 版本編號
    version='0.0.0',
    # 使用find_package自動發現套件，排除test目錄
    packages=find_packages(exclude=['test']),
    # 定義需要安裝到的目錄和檔案清單
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    # 安裝本套件相依的其他套件
    install_requires=['setuptools'],
    # 指定該套件是否可以被安全的作為zip檔案安裝
    zip_safe=True,
    # 維護者姓名
    maintainer='Chen Bo An',
    # 維護者電子郵件
    maintainer_email='andy050629@gmail.com',
    # 套件描述，按照功能套件實例描述
    description='Real Robot Package',
    # 許可證宣告
    license='Apache-2.0',
    # 使用pytest來執行測試
    tests_require=['pytest'],
    # 定義命令列指令稿進入點
    entry_points={
        # 可以用命令列工具
        'console_scripts': [
            'scan_fixer = car.scan_fixer:main',
            'esp32_bridge = car.esp32_bridge:main',
            'odom_pub = car.odom_pub:main',
            'tf_pub = car.tf_pub:main',
        ],
    },
)