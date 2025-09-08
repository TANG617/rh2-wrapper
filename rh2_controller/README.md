# RH2 Controller ROS2 Package

ROS2包装器，用于控制RH2机器人手。支持同时控制左右手，通过PCAN-USB接口进行通信。

## 功能特点

- 支持双手同时控制
- 实时状态反馈
- 电机位置、速度和电流控制
- 标准ROS2接口

## 安装

```bash
# 克隆到工作空间
cd ~/workspace/src
git clone https://your-repo-url/rh2_controller.git

# 编译
cd ~/workspace
colcon build --base-path src

# 加载环境
source install/setup.bash
```

## 使用方法

1. 启动节点：
```bash
ros2 run rh2_controller rh2_dual_hand_node
```

2. 使用launch文件启动：
```bash
ros2 launch rh2_controller rh2_controller.launch.py
```

## Topics

右手：
- `/ry_hand/right/set_angles` (订阅) - 控制指令
- `/ry_hand/right/joint_states` (发布) - 关节状态

左手：
- `/ry_hand/left/set_angles` (订阅) - 控制指令
- `/ry_hand/left/joint_states` (发布) - 关节状态

## 依赖

- ROS2 Humble
- Python 3.8+
- PCAN-USB驱动
