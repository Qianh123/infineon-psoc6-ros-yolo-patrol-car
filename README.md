# 基于英飞凌PSoC6的ROS远程巡检车与YOLO目标识别

本仓库整理了嵌入式比赛作品的三部分核心代码：英飞凌 PSoC6 RT-Thread 底层控制、Orange Pi ROS2/YOLO 上位控制，以及微信小程序远程控制端。

## 仓库结构

```text
infineon-psoc6-ros-yolo-patrol-car
├── psoc6_rtthread
├── orange_pi_ros2_yolo
├── wechat_miniprogram
├── docs
├── README.md
├── LICENSE
└── .gitignore
```

## 系统组成

- 英飞凌 PSoC6：RT-Thread、串口指令解析、电机 PWM 控制、方向控制、超声波测距、安全急停
- Orange Pi：Ubuntu、ROS2、YOLO 视觉识别、摄像头采集、HTTP 接口、串口控制
- 微信小程序：视频显示、状态显示、手动/自主模式切换、速度选择、急停和恢复巡检

## 主要功能

- ROS 远程控制
- YOLO 目标识别
- 超声波避障
- 小程序视频查看
- 小程序手动控制
- 手动/自主巡检模式切换
- 急停与恢复巡检

## 关键接口

视频服务：

```text
http://172.20.10.4:8090/stream.mjpg
http://172.20.10.4:8090/frame.jpg
```

状态与控制接口：

```text
http://172.20.10.4:8000/api/status
http://172.20.10.4:8000/api/mode/manual
http://172.20.10.4:8000/api/mode/auto
http://172.20.10.4:8000/api/manual/move
http://172.20.10.4:8000/api/stop
http://172.20.10.4:8000/api/resume
```

## 启动方式

在 Orange Pi 上进入 ROS2 工作空间并启动巡检程序：

```bash
cd ~/ros2_car_ws
source /opt/ros/humble/setup.bash
source ~/ros2_car_ws/install/setup.bash
./start_patrol.sh
```

## 硬件连接

超声波模块连接：

```text
Trig = D6 / P11_4
Echo = D7 / P11_5
VCC  = 5V
GND  = GND
```

## 目录说明

- `psoc6_rtthread/`：PSoC6 RT-Thread 工程源码，包含 `applications/gpio_probe.c`、板级配置、HAL 驱动、PWM/串口/超声波相关控制代码。
- `orange_pi_ros2_yolo/`：Orange Pi 端 ROS2 Python 包和 `start_patrol.sh` 启动脚本。
- `wechat_miniprogram/`：微信小程序源码，包含页面、组件、工具函数和小程序配置。
- `docs/`：开源整理说明和后续文档。

## 注意事项

本仓库不包含训练数据集、大模型权重、视频文件和编译产物。

如需运行 YOLO 模型，请自行放置模型文件并修改代码中的模型路径。

请根据实际网络环境修改 Orange Pi IP、串口设备名、摄像头设备和模型路径。

## License

本项目使用 MIT License，详见 `LICENSE`。
