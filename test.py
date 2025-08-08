#!/usr/bin/env python3
"""
睿研智能灵巧手 RH2 控制程序
硬件  : PEAK PCAN-USB
驱动  : peak-linux-driver
库    : python-can           pip install python-can
协议  : 力位混合工作模式控制

多电机支持:
- 支持6个电机，ID为1-6
- 对应回复ID为101-106
- 支持单独控制和批量控制

支持的指令解码:
0xA0 - 读取电机状态 (位置、速度、电流、状态)
0xA1 - 读取电机位置
0xA2 - 读取电机速度  
0xA3 - 读取电机电流
0xA4 - 读取电机温度
0xA5 - 读取电机错误状态 (包含详细错误信息解析)
0xA6 - 读取PWM值
0xA7 - 读取编码器值
0xA8 - 读取编码器原始值
0xA9 - 读取电机加速度
0xAA - 读取电机减速度
0xAB - 读取电机最大速度
0xAC - 读取电机最大电流
0xAE - 读取电机最大位置
0xAF - 读取电机最小位置
0xB0 - 读取电机最大加速度
0xB1 - 读取电机最大减速度
0xB2 - 读取电机PID参数(P,I)
0xB3 - 读取电机PID参数(D)
0xB4 - 读取电机使能状态
0xB5 - 读取电机工作模式
0xB6 - 读取电机参数
0xB7 - 读取RS485状态

错误状态位解析:
bit0 - 过流保护
bit1 - 过压保护  
bit2 - 欠压保护
bit3 - 过温保护
bit4 - 堵转保护
bit5 - 编码器错误
bit6 - 通信错误
bit7 - 其他错误

工作模式:
0 - 空闲模式
1 - 位置模式
2 - 速度模式
3 - 电流模式
4 - 力位混合模式

使用方法:
1. 单独控制: controller.send_force_position_command(pos, speed, current, motor_id=1)
2. 批量控制: controller.send_command_to_all_motors(pos, speed, current)
3. 读取状态: controller.read_all_motors_status(command=0xA0)
"""

import can
import time
import struct

class RH2Controller:
    def __init__(self, interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000):
        """初始化RH2控制器"""
        self.bus = can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            state=can.BusState.ACTIVE
        )
        # 支持6个电机，ID为1-6，回复ID为101-106
        self.motor_ids = list(range(1, 7))  # [1, 2, 3, 4, 5, 6]
        self.response_ids = list(range(101, 107))  # [101, 102, 103, 104, 105, 106]
        self.motor_id = 0x01  # 默认电机ID
        self.response_id = 0x101  # 默认应答ID
    
    def set_motor_id(self, motor_id):
        """设置当前操作的电机ID"""
        if motor_id in self.motor_ids:
            self.motor_id = motor_id
            self.response_id = motor_id + 100  # 计算对应的回复ID
            print(f"设置电机ID: {motor_id}, 回复ID: {self.response_id}")
            return True
        else:
            print(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
            return False
        
    def send_force_position_command(self, target_position, speed, current_limit, motor_id=None):
        """
        发送力位混合工作模式指令
        target_position: 目标位置 (0-4095)
        speed: 过程速度 (建议不超过5000)
        current_limit: 最大电流限制 (建议不超过3000)
        motor_id: 电机ID (1-6)，如果为None则使用当前设置的ID
        """
        # 如果指定了电机ID，则设置它
        if motor_id is not None:
            if not self.set_motor_id(motor_id):
                return False
        
        # 构造指令数据: aa + 位置(2字节) + 速度(2字节) + 电流(2字节)
        data = [
            0xAA,  # 力位混合工作模式
            target_position & 0xFF,  # 位置低字节
            (target_position >> 8) & 0xFF,  # 位置高字节
            speed & 0xFF,  # 速度低字节
            (speed >> 8) & 0xFF,  # 速度高字节
            current_limit & 0xFF,  # 电流低字节
            (current_limit >> 8) & 0xFF,  # 电流高字节
            0x00  # 填充字节
        ]
        
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            print(f"发送力位混合指令成功 (电机ID: {self.motor_id}): {msg}")
            print(f"  目标位置: {target_position}")
            print(f"  过程速度: {speed}")
            print(f"  电流限制: {current_limit}")
            return True
        except can.CanError as e:
            print(f"发送失败: {e}")
            return False
    
    def read_motor_status(self, timeout=1.0, motor_id=None):
        """读取电机状态信息"""
        # 如果指定了电机ID，则设置它
        if motor_id is not None:
            if not self.set_motor_id(motor_id):
                return None
        
        print(f"等待应答帧 (电机ID: {self.motor_id}, 回复ID: 0x{self.response_id:03X})...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            rx = self.bus.recv(timeout=0.1)
            if rx is None:
                continue
                
            if rx.arbitration_id == self.response_id:
                return self.decode_status_response(rx)
        
        print(f"未收到电机 {self.motor_id} 的应答")
        return None
    
    def decode_status_response(self, msg):
        """解码状态应答数据"""
        if len(msg.data) < 8:
            print(f"数据长度不足: {len(msg.data)}")
            return None
            
        data = msg.data
        
        # 指令类型映射表
        command_names = {
            0xA0: "读取电机状态",
            0xA1: "读取电机位置", 
            0xA2: "读取电机速度",
            0xA3: "读取电机电流",
            0xA4: "读取电机温度",
            0xA5: "读取电机错误状态",
            0xA6: "读取PWM值",
            0xA7: "读取编码器值",
            0xA8: "读取编码器原始值",
            0xA9: "读取电机加速度",
            0xAA: "读取电机减速度",
            0xAB: "读取电机最大速度",
            0xAC: "读取电机最大电流",
            0xAE: "读取电机最大位置",
            0xAF: "读取电机最小位置",
            0xB0: "读取电机最大加速度",
            0xB1: "读取电机最大减速度",
            0xB2: "读取电机PID参数(P,I)",
            0xB3: "读取电机PID参数(D)",
            0xB4: "读取电机使能状态",
            0xB5: "读取电机工作模式",
            0xB6: "读取电机参数",
            0xB7: "读取RS485状态"
        }
        
        # 根据协议解码数据
        decoded = {
            'raw_data': [hex(x) for x in data],
            'command_type': hex(data[0]),
            'command_name': command_names.get(data[0], f"未知指令({hex(data[0])})"),
            'timestamp': time.time()
        }
        
        print(f"\n=== 解码指令: {decoded['command_name']} (0x{data[0]:02X}) ===")
        print(f"原始数据: {' '.join([f'{x:02X}' for x in data])}")
        
        # 根据不同的指令类型解码
        try:
            if data[0] == 0xA0:  # 读取电机状态
                position = data[1] | (data[2] << 8)
                speed = data[3] | (data[4] << 8)
                current = data[5] | (data[6] << 8)
                status = data[7]
                
                decoded.update({
                    'current_position': position,
                    'current_speed': speed,
                    'current_current': current,
                    'status': status,
                    'position_units': '编码器单位',
                    'speed_units': '编码器单位/秒',
                    'current_units': 'mA'
                })
                
                print(f"电机状态解码结果:")
                print(f"  当前位置: {position} (编码器单位)")
                print(f"  当前速度: {speed} (编码器单位/秒)")
                print(f"  当前电流: {current} mA")
                print(f"  状态标志: 0x{status:02X}")
                
            elif data[0] == 0xA1:  # 读取电机位置
                position = data[1] | (data[2] << 8)
                decoded.update({
                    'current_position': position,
                    'position_units': '编码器单位'
                })
                print(f"电机位置: {position} (编码器单位)")
                
            elif data[0] == 0xA2:  # 读取电机速度
                speed = data[1] | (data[2] << 8)
                decoded.update({
                    'current_speed': speed,
                    'speed_units': '编码器单位/秒'
                })
                print(f"电机速度: {speed} (编码器单位/秒)")
                
            elif data[0] == 0xA3:  # 读取电机电流
                current = data[1] | (data[2] << 8)
                decoded.update({
                    'current_current': current,
                    'current_units': 'mA'
                })
                print(f"电机电流: {current} mA")
                
            elif data[0] == 0xA4:  # 读取电机温度
                temperature = data[1]
                decoded.update({
                    'temperature': temperature,
                    'temperature_units': '°C'
                })
                print(f"电机温度: {temperature}°C")
                
            elif data[0] == 0xA5:  # 读取电机错误状态
                error_status = data[1]
                error_messages = []
                
                # 解析错误状态位
                if error_status & 0x01:
                    error_messages.append("过流保护")
                if error_status & 0x02:
                    error_messages.append("过压保护")
                if error_status & 0x04:
                    error_messages.append("欠压保护")
                if error_status & 0x08:
                    error_messages.append("过温保护")
                if error_status & 0x10:
                    error_messages.append("堵转保护")
                if error_status & 0x20:
                    error_messages.append("编码器错误")
                if error_status & 0x40:
                    error_messages.append("通信错误")
                if error_status & 0x80:
                    error_messages.append("其他错误")
                
                decoded.update({
                    'error_status': error_status,
                    'error_messages': error_messages,
                    'has_error': error_status != 0
                })
                
                if error_messages:
                    print(f"电机错误状态: 0x{error_status:02X} - {', '.join(error_messages)}")
                else:
                    print(f"电机状态正常: 0x{error_status:02X}")
                    
            elif data[0] == 0xA6:  # 读取PWM值
                pwm_value = data[1] | (data[2] << 8)
                decoded.update({
                    'pwm_value': pwm_value,
                    'pwm_units': 'PWM单位'
                })
                print(f"PWM值: {pwm_value} (PWM单位)")
                
            elif data[0] == 0xA7:  # 读取编码器值
                encoder_value = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24)
                decoded.update({
                    'encoder_value': encoder_value,
                    'encoder_units': '编码器单位'
                })
                print(f"编码器值: {encoder_value} (编码器单位)")
                
            elif data[0] == 0xA8:  # 读取编码器原始值
                raw_encoder_value = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24)
                decoded.update({
                    'raw_encoder_value': raw_encoder_value,
                    'encoder_units': '原始编码器单位'
                })
                print(f"编码器原始值: {raw_encoder_value} (原始编码器单位)")
                
            elif data[0] == 0xA9:  # 读取电机加速度
                acceleration = data[1] | (data[2] << 8)
                decoded.update({
                    'acceleration': acceleration,
                    'acceleration_units': '编码器单位/秒²'
                })
                print(f"电机加速度: {acceleration} (编码器单位/秒²)")
                
            elif data[0] == 0xAA:  # 读取电机减速度
                deceleration = data[1] | (data[2] << 8)
                decoded.update({
                    'deceleration': deceleration,
                    'deceleration_units': '编码器单位/秒²'
                })
                print(f"电机减速度: {deceleration} (编码器单位/秒²)")
                
            elif data[0] == 0xAB:  # 读取电机最大速度
                max_speed = data[1] | (data[2] << 8)
                decoded.update({
                    'max_speed': max_speed,
                    'speed_units': '编码器单位/秒'
                })
                print(f"电机最大速度: {max_speed} (编码器单位/秒)")
                
            elif data[0] == 0xAC:  # 读取电机最大电流
                max_current = data[1] | (data[2] << 8)
                decoded.update({
                    'max_current': max_current,
                    'current_units': 'mA'
                })
                print(f"电机最大电流: {max_current} mA")
                
            elif data[0] == 0xAE:  # 读取电机最大位置
                max_position = data[1] | (data[2] << 8)
                decoded.update({
                    'max_position': max_position,
                    'position_units': '编码器单位'
                })
                print(f"电机最大位置: {max_position} (编码器单位)")
                
            elif data[0] == 0xAF:  # 读取电机最小位置
                min_position = data[1] | (data[2] << 8)
                decoded.update({
                    'min_position': min_position,
                    'position_units': '编码器单位'
                })
                print(f"电机最小位置: {min_position} (编码器单位)")
                
            elif data[0] == 0xB0:  # 读取电机最大加速度
                max_acceleration = data[1] | (data[2] << 8)
                decoded.update({
                    'max_acceleration': max_acceleration,
                    'acceleration_units': '编码器单位/秒²'
                })
                print(f"电机最大加速度: {max_acceleration} (编码器单位/秒²)")
                
            elif data[0] == 0xB1:  # 读取电机最大减速度
                max_deceleration = data[1] | (data[2] << 8)
                decoded.update({
                    'max_deceleration': max_deceleration,
                    'deceleration_units': '编码器单位/秒²'
                })
                print(f"电机最大减速度: {max_deceleration} (编码器单位/秒²)")
                
            elif data[0] == 0xB2:  # 读取电机PID参数(P,I)
                if len(data) >= 9:
                    pid_p = struct.unpack('<f', bytes(data[1:5]))[0]
                    pid_i = struct.unpack('<f', bytes(data[5:9]))[0]
                    decoded.update({
                        'pid_p': pid_p,
                        'pid_i': pid_i,
                        'pid_units': '无单位'
                    })
                    print(f"PID参数: P={pid_p:.6f}, I={pid_i:.6f}")
                else:
                    print("数据长度不足，无法解码PID参数")
                    
            elif data[0] == 0xB3:  # 读取电机PID参数(D)
                if len(data) >= 5:
                    pid_d = struct.unpack('<f', bytes(data[1:5]))[0]
                    decoded.update({
                        'pid_d': pid_d,
                        'pid_units': '无单位'
                    })
                    print(f"PID参数: D={pid_d:.6f}")
                else:
                    print("数据长度不足，无法解码PID参数")
                    
            elif data[0] == 0xB4:  # 读取电机使能状态
                enabled = data[1] == 1
                decoded.update({
                    'enabled': enabled,
                    'enabled_text': '已使能' if enabled else '未使能'
                })
                print(f"电机使能状态: {'已使能' if enabled else '未使能'}")
                
            elif data[0] == 0xB5:  # 读取电机工作模式
                work_mode = data[1]
                mode_names = {
                    0: "空闲模式",
                    1: "位置模式", 
                    2: "速度模式",
                    3: "电流模式",
                    4: "力位混合模式"
                }
                mode_name = mode_names.get(work_mode, f"未知模式({work_mode})")
                decoded.update({
                    'work_mode': work_mode,
                    'work_mode_name': mode_name
                })
                print(f"电机工作模式: {mode_name} (模式{work_mode})")
                
            elif data[0] == 0xB6:  # 读取电机参数
                parameter_count = data[1]
                parameters = data[2:8]  # 6个参数
                decoded.update({
                    'parameter_count': parameter_count,
                    'parameters': parameters,
                    'parameter_units': '参数单位'
                })
                print(f"电机参数: 数量={parameter_count}")
                for i, param in enumerate(parameters):
                    print(f"  参数{i+1}: {param}")
                    
            elif data[0] == 0xB7:  # 读取RS485状态
                rs485_status = data[1]
                decoded.update({
                    'rs485_status': rs485_status,
                    'rs485_hex': f"0x{rs485_status:02X}"
                })
                print(f"RS485状态: 0x{rs485_status:02X}")
                
            else:
                print(f"未知指令类型: 0x{data[0]:02X}")
                decoded['error'] = f"未知指令类型: 0x{data[0]:02X}"
                
        except Exception as e:
            print(f"解码过程中发生错误: {e}")
            decoded['error'] = str(e)
            
        print("=" * 50)
        return decoded
    
    def send_read_command(self, command, motor_id=None):
        """发送读取指令"""
        # 如果指定了电机ID，则设置它
        if motor_id is not None:
            if not self.set_motor_id(motor_id):
                return False
        
        data = [command] + [0x00] * 7  # 填充剩余字节为0
        
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            print(f"发送读取指令成功 (电机ID: {self.motor_id}): 0x{command:02X}")
            return True
        except can.CanError as e:
            print(f"发送失败: {e}")
            return False
    
    def send_command_to_all_motors(self, target_position, speed, current_limit):
        """向所有6个电机发送相同的力位混合指令"""
        print(f"\n=== 向所有电机发送指令 ===")
        results = []
        for motor_id in self.motor_ids:
            print(f"\n发送指令到电机 {motor_id}:")
            success = self.send_force_position_command(
                target_position, speed, current_limit, motor_id
            )
            results.append((motor_id, success))
            time.sleep(0.1)  # 短暂延时避免总线冲突
        return results
    
    def read_all_motors_status(self, command=0xA0):
        """读取所有6个电机的状态"""
        print(f"\n=== 读取所有电机状态 ===")
        results = {}
        for motor_id in self.motor_ids:
            print(f"\n读取电机 {motor_id} 状态:")
            if self.send_read_command(command, motor_id):
                status = self.read_motor_status(motor_id=motor_id)
                results[motor_id] = status
            time.sleep(0.1)  # 短暂延时
        return results
    
    def close(self):
        """关闭CAN总线"""
        self.bus.shutdown()
    
    def test_decode_with_sample_data(self):
        """使用示例数据测试解码功能"""
        print("\n=== 解码功能测试 ===")
        
        # 模拟各种指令的响应数据
        test_cases = [
            # 0xA0 - 读取电机状态
            can.Message(arbitration_id=0x101, data=[0xA0, 0xE8, 0x03, 0x64, 0x00, 0x32, 0x00, 0x00]),
            # 0xA1 - 读取电机位置  
            can.Message(arbitration_id=0x101, data=[0xA1, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0xA2 - 读取电机速度
            can.Message(arbitration_id=0x101, data=[0xA2, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0xA3 - 读取电机电流
            can.Message(arbitration_id=0x101, data=[0xA3, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0xA4 - 读取电机温度
            can.Message(arbitration_id=0x101, data=[0xA4, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0xA5 - 读取电机错误状态 (正常)
            can.Message(arbitration_id=0x101, data=[0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0xA5 - 读取电机错误状态 (有过流错误)
            can.Message(arbitration_id=0x101, data=[0xA5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0xB4 - 读取电机使能状态 (已使能)
            can.Message(arbitration_id=0x101, data=[0xB4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0xB5 - 读取电机工作模式 (力位混合模式)
            can.Message(arbitration_id=0x101, data=[0xB5, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        ]
        
        for i, test_msg in enumerate(test_cases, 1):
            print(f"\n测试用例 {i}:")
            self.decode_status_response(test_msg)
            time.sleep(0.1)

def main():
    """主函数 - 演示RH2控制功能"""
    controller = RH2Controller()
    
    try:
        print("=== RH2 灵巧手控制演示 ===")
        
        # 首先进行解码功能测试
        controller.test_decode_with_sample_data()
        
        # 示例1: 向所有电机发送力位混合指令 - 移动到位置0
        print("\n1. 向所有电机发送力位混合指令 - 移动到位置0")
        controller.send_command_to_all_motors(
            target_position=0,      # 目标位置0
            speed=2530,             # 过程速度2.53个行程/秒
            current_limit=868       # 最大电流868mA
        )
        time.sleep(1.0)
        
        # 示例2: 向所有电机发送力位混合指令 - 移动到位置4095
        print("\n2. 向所有电机发送力位混合指令 - 移动到位置4095")
        controller.send_command_to_all_motors(
            target_position=4095,   # 目标位置4095
            speed=2530,             # 过程速度2.53个行程/秒
            current_limit=868       # 最大电流868mA
        )
        time.sleep(1.0)

                # 示例2: 向所有电机发送力位混合指令 - 移动到位置0
        print("\n2. 向所有电机发送力位混合指令 - 移动到位置4095")
        controller.send_command_to_all_motors(
            target_position=0,   # 目标位置0
            speed=2530,             # 过程速度2.53个行程/秒
            current_limit=868       # 最大电流868mA
        )
        time.sleep(1.0)
        
        # 示例3: 读取所有电机状态
        print("\n3. 读取所有电机状态")
        all_status = controller.read_all_motors_status(0xA0)
        print(f"所有电机状态: {all_status}")
        
        # 示例4: 读取所有电机位置
        print("\n4. 读取所有电机位置")
        all_positions = controller.read_all_motors_status(0xA1)
        print(f"所有电机位置: {all_positions}")
        
        # 示例5: 读取所有电机速度
        print("\n5. 读取所有电机速度")
        all_speeds = controller.read_all_motors_status(0xA2)
        print(f"所有电机速度: {all_speeds}")
        
        # 示例6: 读取所有电机电流
        print("\n6. 读取所有电机电流")
        all_currents = controller.read_all_motors_status(0xA3)
        print(f"所有电机电流: {all_currents}")
        
        # 示例7: 读取所有电机温度
        print("\n7. 读取所有电机温度")
        all_temperatures = controller.read_all_motors_status(0xA4)
        print(f"所有电机温度: {all_temperatures}")
        
        # 示例8: 读取所有电机错误状态
        print("\n8. 读取所有电机错误状态")
        all_errors = controller.read_all_motors_status(0xA5)
        print(f"所有电机错误状态: {all_errors}")
        
        # 示例9: 单独控制每个电机到不同位置
        print("\n9. 单独控制每个电机到不同位置")
        positions = [0, 1000, 2000, 3000, 3500, 4095]  # 每个电机到不同位置
        for i, motor_id in enumerate(controller.motor_ids):
            print(f"\n控制电机 {motor_id} 到位置 {positions[i]}:")
            controller.send_force_position_command(
                target_position=positions[i],
                speed=2530,
                current_limit=868,
                motor_id=motor_id
            )
            time.sleep(0.2)
        
        print("\n=== 演示完成 ===")
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        controller.close()
        print("CAN总线已关闭")

if __name__ == "__main__":
    main()