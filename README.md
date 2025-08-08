# RH2 睿研智能灵巧手控制器库

这是一个用于控制睿研智能灵巧手RH2的Python库，支持通过CAN总线控制6个电机。

## 功能特性

- ✅ 支持6个电机控制（ID: 1-6，回复ID: 101-106）
- ✅ 力位混合工作模式控制
- ✅ 完整的指令解码功能（0xA0-0xB7）
- ✅ 单独控制和批量控制
- ✅ 实时状态监控
- ✅ 错误状态解析
- ✅ 上下文管理器支持
- ✅ 类型提示支持

## 硬件要求

- PEAK PCAN-USB 适配器
- peak-linux-driver 驱动
- python-can 库

## 安装依赖

```bash
pip install python-can
```

## 快速开始

### 基本使用

```python
from rh2_controller import RH2Controller

# 使用上下文管理器（推荐）
with RH2Controller() as controller:
    if controller.is_connected():
        # 移动电机1到位置1000
        controller.move_motor_to_position(1, 1000)
        
        # 获取电机1的位置
        position = controller.get_motor_position(1)
        print(f"电机1位置: {position}")
```

### 批量控制

```python
from rh2_controller import RH2Controller

with RH2Controller() as controller:
    # 移动所有电机到位置0
    results = controller.move_all_motors_to_position(0)
    
    # 获取所有电机位置
    positions = controller.get_all_motors_positions()
    for motor_id, pos in positions.items():
        print(f"电机{motor_id}: {pos}")
```

### 状态监控

```python
from rh2_controller import RH2Controller

with RH2Controller() as controller:
    # 获取所有电机状态
    all_status = controller.read_all_motors_status()
    
    # 获取所有电机温度
    temperatures = controller.get_all_motors_temperatures()
    
    # 获取所有电机错误状态
    error_status = controller.get_all_motors_error_status()
```

## API 参考

### 主要类

#### RH2Controller

主要的控制器类，提供所有控制功能。

**构造函数参数：**
- `interface`: CAN接口类型，默认 'pcan'
- `channel`: CAN通道名称，默认 'PCAN_USBBUS1'
- `bitrate`: CAN总线波特率，默认 1000000
- `auto_connect`: 是否自动连接，默认 True

### 主要方法

#### 连接管理

- `connect()`: 连接CAN总线
- `disconnect()`: 断开CAN总线连接
- `is_connected()`: 检查连接状态

#### 电机控制

- `move_motor_to_position(motor_id, target_position, speed=2530, current_limit=868)`: 移动指定电机到目标位置
- `move_all_motors_to_position(target_position, speed=2530, current_limit=868)`: 移动所有电机到目标位置
- `send_force_position_command(target_position, speed, current_limit, motor_id=None)`: 发送力位混合指令

#### 状态读取

- `get_motor_position(motor_id)`: 获取指定电机位置
- `get_motor_speed(motor_id)`: 获取指定电机速度
- `get_motor_current(motor_id)`: 获取指定电机电流
- `get_motor_temperature(motor_id)`: 获取指定电机温度
- `get_motor_error_status(motor_id)`: 获取指定电机错误状态

#### 批量读取

- `get_all_motors_positions()`: 获取所有电机位置
- `get_all_motors_speeds()`: 获取所有电机速度
- `get_all_motors_currents()`: 获取所有电机电流
- `get_all_motors_temperatures()`: 获取所有电机温度
- `get_all_motors_error_status()`: 获取所有电机错误状态

#### 高级功能

- `read_all_motors_status(command=0xA0)`: 读取所有电机状态
- `send_read_command(command, motor_id=None)`: 发送读取指令
- `read_motor_status(timeout=1.0, motor_id=None)`: 读取电机状态

### 指令常量

```python
# 读取指令
COMMAND_READ_STATUS = 0xA0        # 读取电机状态
COMMAND_READ_POSITION = 0xA1      # 读取电机位置
COMMAND_READ_SPEED = 0xA2         # 读取电机速度
COMMAND_READ_CURRENT = 0xA3       # 读取电机电流
COMMAND_READ_TEMPERATURE = 0xA4   # 读取电机温度
COMMAND_READ_ERROR = 0xA5         # 读取电机错误状态
# ... 更多指令

# 控制指令
COMMAND_FORCE_POSITION = 0xAA     # 力位混合模式
```

## 使用示例

### 示例1: 基本控制

```python
from rh2_controller import RH2Controller
import time

with RH2Controller() as controller:
    # 移动电机1到位置1000
    controller.move_motor_to_position(1, 1000)
    time.sleep(1)
    
    # 获取位置
    pos = controller.get_motor_position(1)
    print(f"电机1位置: {pos}")
```

### 示例2: 批量控制

```python
from rh2_controller import RH2Controller

with RH2Controller() as controller:
    # 所有电机到位置0
    controller.move_all_motors_to_position(0)
    
    # 获取所有位置
    positions = controller.get_all_motors_positions()
    for motor_id, pos in positions.items():
        print(f"电机{motor_id}: {pos}")
```

### 示例3: 状态监控

```python
from rh2_controller import RH2Controller

with RH2Controller() as controller:
    # 获取完整状态
    all_status = controller.read_all_motors_status()
    
    for motor_id, status in all_status.items():
        if status:
            print(f"电机{motor_id}:")
            print(f"  位置: {status.get('current_position')}")
            print(f"  速度: {status.get('current_speed')}")
            print(f"  电流: {status.get('current_current')}")
```

### 示例4: 错误处理

```python
from rh2_controller import RH2Controller

with RH2Controller() as controller:
    # 检查错误状态
    error_status = controller.get_all_motors_error_status()
    
    for motor_id, error in error_status.items():
        if error and 'error_messages' in error:
            if error['error_messages']:
                print(f"电机{motor_id}错误: {', '.join(error['error_messages'])}")
            else:
                print(f"电机{motor_id}: 正常")
```

## 错误状态说明

错误状态位解析：
- bit0: 过流保护
- bit1: 过压保护
- bit2: 欠压保护
- bit3: 过温保护
- bit4: 堵转保护
- bit5: 编码器错误
- bit6: 通信错误
- bit7: 其他错误

## 工作模式

- 0: 空闲模式
- 1: 位置模式
- 2: 速度模式
- 3: 电流模式
- 4: 力位混合模式

## 注意事项

1. 确保CAN总线连接正常
2. 电机ID范围为1-6
3. 位置范围为0-4095
4. 速度建议不超过5000
5. 电流限制建议不超过3000
6. 使用上下文管理器可以自动管理连接

## 运行测试

```bash
# 运行库测试
python rh2_controller.py

# 运行使用示例
python example_usage.py
```

## 许可证

MIT License 