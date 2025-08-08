# RH2 Controller ROS2 包装器

RH2智能灵巧手的ROS2接口包装器，提供完整的电机控制和状态查询功能。

## 功能特性

- ✅ **电机信息查询** - 获取位置、速度、电流和状态
- ✅ **电机运动控制** - 批量或单独控制电机运动
- ✅ **ROS2标准接口** - 使用JointState消息格式
- ✅ **实时状态发布** - 10Hz频率发布电机状态
- ✅ **错误处理** - 完整的错误检测和响应
- ❌ **触觉传感器** - 暂未实现（按需求）

## 依赖要求

### 系统依赖
```bash
# ROS2 (Humble/Iron/Rolling)
sudo apt install ros-<distro>-desktop

# Python依赖
pip install python-can
```

### 硬件依赖
- PEAK PCAN-USB设备
- peak-linux-driver

## 安装

1. **克隆到工作空间**
```bash
cd ~/ros2_ws/src
git clone <repository-url> rh2_controller
```

2. **编译包**
```bash
cd ~/ros2_ws
colcon build --packages-select rh2_controller
source install/setup.bash
```

## 使用方法

### 1. 启动RH2控制器节点

```bash
# 使用默认参数启动
ros2 run rh2_controller rh2_controller_node

# 或使用launch文件启动
ros2 launch rh2_controller rh2_controller.launch.py

# 自定义参数启动
ros2 launch rh2_controller rh2_controller.launch.py \
    interface:=pcan \
    channel:=PCAN_USBBUS1 \
    bitrate:=1000000 \
    motor_ids:="[1,2,3,4,5,6]"
```

### 2. 查看话题

```bash
# 查看可用话题
ros2 topic list

# 查看响应话题
ros2 topic echo /rh2/response

# 查看话题信息
ros2 topic info /rh2/controller
ros2 topic info /rh2/response
```

### 3. 发送控制指令

#### 获取电机信息
```bash
ros2 topic pub --once /rh2/controller sensor_msgs/msg/JointState \
'{
  header: {frame_id: "rh2_command"},
  name: ["get_motors_info"],
  position: [],
  velocity: [], 
  effort: []
}'
```

#### 移动电机
```bash
ros2 topic pub --once /rh2/controller sensor_msgs/msg/JointState \
'{
  header: {frame_id: "rh2_command"},
  name: ["move_motors"],
  position: [2048.0, 2048.0, 2048.0, 2048.0, 2048.0, 2048.0],
  velocity: [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0],
  effort: [800.0, 800.0, 800.0, 800.0, 800.0, 800.0]
}'
```

### 4. 运行测试客户端

```bash
# 启动测试客户端（自动运行测试序列）
ros2 run rh2_controller rh2_test_client
```

## 话题接口

### 订阅话题: `/rh2/controller`
**消息类型**: `sensor_msgs/msg/JointState`

#### 获取电机信息格式:
```yaml
header:
  frame_id: "rh2_command"
name: ["get_motors_info"]
position: []
velocity: []
effort: []
```

#### 移动电机格式:
```yaml
header:
  frame_id: "rh2_command"  
name: ["move_motors"]
position: [pos1, pos2, pos3, pos4, pos5, pos6]  # 目标位置 (0-4095)
velocity: [vel1, vel2, vel3, vel4, vel5, vel6]  # 速度
effort: [eff1, eff2, eff3, eff4, eff5, eff6]    # 电流限制
```

### 发布话题: `/rh2/response`
**消息类型**: `sensor_msgs/msg/JointState`

#### 正常响应格式:
```yaml
header:
  frame_id: "rh2_motors"
name: ["command_type", "motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"]
position: [pos1, pos2, pos3, pos4, pos5, pos6]  # 当前位置
velocity: [vel1, vel2, vel3, vel4, vel5, vel6]  # 当前速度
effort: [cur1, cur2, cur3, cur4, cur5, cur6]    # 当前电流(mA)
```

#### 错误响应格式:
```yaml
header:
  frame_id: "rh2_motors"
name: ["error", "错误信息描述"]
position: []
velocity: []
effort: []
```

## Python客户端示例

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RH2Client(Node):
    def __init__(self):
        super().__init__('rh2_client')
        
        # 创建发布者和订阅者
        self.pub = self.create_publisher(JointState, '/rh2/controller', 10)
        self.sub = self.create_subscription(JointState, '/rh2/response', 
                                          self.response_callback, 10)
    
    def response_callback(self, msg):
        """处理响应"""
        if msg.name[0] == "error":
            print(f"错误: {msg.name[1]}")
        else:
            print(f"命令: {msg.name[0]}")
            for i, motor in enumerate(msg.name[1:]):
                if i < len(msg.position):
                    print(f"  {motor}: pos={msg.position[i]}")
    
    def get_motors_info(self):
        """获取电机信息"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["get_motors_info"]
        msg.position = []
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)
    
    def move_motors(self, positions, speeds, currents):
        """移动电机"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["move_motors"]
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(s) for s in speeds]
        msg.effort = [float(c) for c in currents]
        self.pub.publish(msg)

def main():
    rclpy.init()
    client = RH2Client()
    
    # 获取电机信息
    client.get_motors_info()
    
    # 移动电机到中间位置
    client.move_motors([2048]*6, [2000]*6, [800]*6)
    
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

## 故障排除

### 1. 连接问题
```bash
# 检查CAN设备
lsmod | grep peak
dmesg | grep pcan

# 检查节点状态
ros2 node list
ros2 node info /rh2_controller
```

### 2. 权限问题
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录生效
```

### 3. 调试模式
```bash
# 启用详细日志
ros2 run rh2_controller rh2_controller_node --ros-args --log-level debug
```

## 参数配置

| 参数 | 默认值 | 描述 |
|------|--------|------|
| interface | pcan | CAN接口类型 |
| channel | PCAN_USBBUS1 | CAN通道 |
| bitrate | 1000000 | 波特率 |
| motor_ids | [1,2,3,4,5,6] | 电机ID列表 |

## 性能说明

- **状态更新频率**: 10Hz
- **指令响应时间**: < 100ms
- **CAN总线负载**: 约30% @ 1Mbps
- **内存使用**: < 50MB

## 注意事项

1. **电机ID范围**: 1-254 (0为广播，255无效)
2. **位置范围**: 0-4095 (对应满行程)
3. **速度单位**: 编码器单位/秒
4. **电流单位**: mA (毫安)
5. **同时运行**: 避免多个节点同时连接同一CAN设备
