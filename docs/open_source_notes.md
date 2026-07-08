# 开源整理说明

本仓库用于提交比赛系统，保留源码和必要工程配置，排除本地构建产物、缓存、模型权重、图片数据集和视频文件。

已整理的主要内容：

- `psoc6_rtthread/`：PSoC6 RT-Thread 底层控制工程。
- `orange_pi_ros2_yolo/`：Orange Pi ROS2/YOLO 控制代码与启动脚本。
- `wechat_miniprogram/`：微信小程序远程控制端代码。

未纳入仓库的内容：

- ROS2 `build/`、`install/`、`log/`
- RT-Thread/Keil 编译产物
- YOLO `.pt`、`.onnx` 模型权重
- 视频文件、运行截图、图片数据集
- 小程序 `node_modules/`、`miniprogram_npm/`、`dist/`
- 本地私有配置和临时缓存
