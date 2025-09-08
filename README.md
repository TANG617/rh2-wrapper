# RH2机械手ROS2包装器

## 📖 项目简介

这是一个用于RH2机械手的ROS2包装器，提供了完整的电机控制、状态监控和模拟功能。支持真实硬件和模拟模式，方便开发和测试。

## ✨ 主要功能

- 🔧 **电机控制**: 支持6个电机的位置、速度、电流控制
- 📊 **状态监控**: 实时获取电机位置、速度、电流等状态信息
- 🤖 **模拟模式**: 无需硬件即可进行开发和测试
- 🌐 **ROS2集成**: 提供标准的ROS2 topic接口
- 📡 **CAN通信**: 支持socketcan、PCAN和其他CAN接口
- 🤖🤖 **双手控制**: 支持左右手独立控制（can0/can1）
- 🎮 **触觉反馈**: 支持触觉传感器数据读取

## 📁 项目结构

```
rh2-wrapper/
├── rh2_controller.py      # 核心控制器类
├── rh2_controller_sim.py  # 模拟器类
├── rh2_ros_wrapper.py     # ROS2包装器
├── rh2_node.py           # 双手控制节点
├── test_client.py        # 测试客户端
├── launch/               # 启动文件
│   ├── rh2_controller.launch.py
│   └── rh2_test.launch.py
├── test/                 # 测试脚本
│   ├── test_motor.py
│   └── test_tactile.py
├── run_sim_node.py       # 模拟节点启动器
├── demo.py              # 完整演示脚本
├── test_sim.py          # 简单测试脚本
└── setup.py             # 包配置
```

## 🚀 快速开始

### 1. 安装依赖

```bash
# 安装Python依赖
pip install python-can>=4.0.0

# 确保ROS2环境已配置
source /opt/ros/humble/setup.bash  # 根据你的ROS2版本调整
```

### 2. 配置CAN接口（真实硬件）

如果使用真实硬件，首先配置CAN接口：

```bash
# 配置CAN接口（需要root权限）
sudo ./setup_can.sh

# 手动配置（可选）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set up can1
```

### 3. 运行测试

#### 模拟模式测试
最简单的方式是使用模拟模式：

```bash
# 运行基础测试
python3 test_sim.py

# 启动模拟节点
python3 run_sim_node.py

# 运行完整演示
python3 demo.py
```

#### 真实硬件测试

```bash
# 测试硬件连接
python3 run_hardware_test.py

# 启动单手控制（can0）
python3 rh2_ros_wrapper.py

# 启动双手控制（can0+can1）
python3 rh2_node.py

# 双手控制演示
python3 demo_dual_hands.py
```


## 🎯 使用方法

### ROS2 Topics

#### 单手控制
- 订阅: `/ry_hand/{hand_name}/set_angles` (sensor_msgs/JointState)
- 发布: `/ry_hand/{hand_name}/joint_states` (sensor_msgs/JointState)

#### 双手控制
- 右手订阅: `/ry_hand/right/set_angles` (sensor_msgs/JointState)
- 右手发布: `/ry_hand/right/joint_states` (sensor_msgs/JointState)
- 左手订阅: `/ry_hand/left/set_angles` (sensor_msgs/JointState)
- 左手发布: `/ry_hand/left/joint_states` (sensor_msgs/JointState)

### 控制命令格式

发送到 `set_angles` 话题的 JointState 消息格式：

```python
msg = JointState()
msg.header.stamp = node.get_clock().now().to_msg()
msg.name = ["command_type"]  # "get_motors_info" 或 "move_motors"

# 对于move_motors命令：
msg.position = [pos1, pos2, pos3, pos4, pos5, pos6]  # 目标位置 (0-4095)
msg.velocity = [vel1, vel2, vel3, vel4, vel5, vel6]  # 速度 
msg.effort = [cur1, cur2, cur3, cur4, cur5, cur6]    # 电流限制
```

### Python API 示例

```python
import rclpy
from sensor_msgs.msg import JointState

# 初始化ROS2
rclpy.init()
node = rclpy.create_node('rh2_client')

# 创建发布者
pub = node.create_publisher(JointState, '/ry_hand/right/set_angles', 10)

# 发送获取信息命令
msg = JointState()
msg.header.stamp = node.get_clock().now().to_msg()
msg.name = ["get_motors_info"]
pub.publish(msg)

# 发送移动命令
msg = JointState()
msg.header.stamp = node.get_clock().now().to_msg()
msg.name = ["move_motors"]
msg.position = [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0]
msg.velocity = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
msg.effort = [500.0, 500.0, 500.0, 500.0, 500.0, 500.0]
pub.publish(msg)
```

## 🔧 配置选项

### 启动参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| interface | socketcan | CAN接口类型 (socketcan/pcan/sim) |
| channel | can0 | CAN通道 (can0/can1/PCAN_USBBUS1等) |
| bitrate | 1000000 | 波特率 |
| motor_ids | [1,2,3,4,5,6] | 电机ID列表 |
| hand_name | right | 手的名称 (left/right) |
| simulate | false | 是否使用模拟模式 |

### CAN接口配置

#### socketcan (推荐)
```bash
# 配置和启动CAN接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

#### 双手配置
- 右手: can0
- 左手: can1

### 模拟模式

模拟模式不需要真实硬件，适合开发和测试：

```python
# 创建模拟控制器
from rh2_controller_sim import RH2ControllerSim
controller = RH2ControllerSim(motor_ids=[1,2,3,4,5,6])

# 或在ROS包装器中使用
wrapper = RH2ROSWrapper(interface='sim', simulate=True)
```

## 🧪 测试

### 单元测试

```bash
# 测试控制器核心功能
python3 test/test_motor.py

# 测试触觉传感器
python3 test/test_tactile.py

# 测试模拟器
python3 test_sim.py
```

### 集成测试

```bash
# 运行完整测试客户端
python3 test_client.py

# 运行演示程序
python3 demo.py
```

## 🐛 故障排除

### 常见问题

1. **CAN连接失败**
   ```bash
   # 检查CAN设备
   lsusb | grep -i peak
   
   # 检查权限
   ls -l /dev/pcan*
   
   # 解决方案：使用模拟模式
   python3 run_sim_node.py
   ```

2. **权限错误**
   ```bash
   # 添加用户到dialout组
   sudo usermod -a -G dialout $USER
   
   # 重新登录或重启
   ```

3. **依赖缺失**
   ```bash
   # 安装python-can
   pip install python-can
   
   # 检查ROS2环境
   echo $ROS_DISTRO
   ```

### 调试模式

启用详细日志：

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 📚 API参考

### RH2Controller 类

```python
controller = RH2Controller(
    interface='pcan',           # CAN接口
    channel='PCAN_USBBUS1',    # CAN通道  
    bitrate=1000000,           # 波特率
    motor_ids=[1,2,3,4,5,6]    # 电机ID列表
)

# 主要方法
controller.get_motors_info()           # 获取电机信息
controller.move_motors(pos, vel, cur)  # 移动电机
controller.get_tactile_data()          # 获取触觉数据
controller.is_connected()              # 检查连接状态
```

### RH2ROSWrapper 类

```python
wrapper = RH2ROSWrapper(
    interface='pcan',
    channel='PCAN_USBBUS1',
    motor_ids=[1,2,3,4,5,6],
    hand_name='right',
    simulate=False
)

# ROS2节点功能
# 自动处理topic订阅和发布
# 提供状态监控和错误处理
```

## 🤝 贡献

欢迎提交issues和pull requests来改进这个项目！

## 📄 许可证

MIT License

## 📞 支持

如果遇到问题，请：
1. 查看本README的故障排除部分
2. 运行测试脚本验证环境
3. 提交issue描述问题

---

🎉 **享受使用RH2机械手进行机器人开发的乐趣！**
