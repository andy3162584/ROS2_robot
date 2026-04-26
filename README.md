car/
├── car/                      # 核心節點原始碼
│   ├── __init__.py
│   ├── esp32_bridge.py       # 與硬體溝通的橋樑
│   ├── odom_pub.py           # 發布里程計數據
│   ├── scan_fixer.py         # 雷達數據優化
│   └── tf_pub.py             # (備用) 原始座標廣播
├── config/                   # 參數設定檔
│   ├── ekf.yaml              # EKF 濾波器融合設定
│   ├── nav2_params.yaml      # 導航參數
│   └── slam_params.yaml      # SLAM 建圖參數
├── launch/                   # 啟動腳本
│   ├── hardware.launch.py    # 啟動雷達與底盤
│   ├── nav2.launch.py        # 啟動導航
│   ├── rsp.launch.py         # 啟動機器人狀態發布器
│   └── slam.launch.py        # 啟動 SLAM 建圖
├── maps/                     # 地圖儲存空間
│   ├── map.pgm
│   └── map.yaml
├── meshes/                   # 機器人 3D 模型檔案
│   └── andy2025S.STL
├── resource/                 # ROS 2 資源索引
│   └── car
├── rviz/                     # 可視化設定檔
│   ├── nav2.rviz             # nav2的rviz設定檔
│   └── slam.rviz             # slam的rviz設定檔
├── urdf/                     # 機器人結構描述
│   └── robot.xacro
├── package.xml               # 套件依賴定義
├── README.md                 # 專案說明文件
├── setup.cfg                 # 編譯設定
└── setup.py                  # 安裝與檔案映射腳本

Python功能套件
    package.xml
    功能套件的版權描述，敘述版權所有者、版權年分、版權宣告、功能套件相依的函式庫、工具或資源
    setup.py
    描述如何安裝和分發功能套件，包含功能套件的中繼資料、相依關係、安裝指令稿等資訊
    car資料夾
        建立需要的節點(node)檔案
        esp32_bridge.py
        與esp32傳輸，並發送motor_pos、imu/data_raw，訂閱cmd_vel
        odom_pub.py
        讀取motor_pos、imu/data_raw，並將其轉換成並發布odom、tf