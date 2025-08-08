#!/usr/bin/env python3
"""
睿研智能灵巧手 RH2 批量控制器库 - 标准CAN版本
硬件  : PEAK PCAN-USB
驱动  : peak-linux-driver  
库    : python-can           pip install python-can
协议  : 根据睿研智能灵巧手通信协议.csv

特性:
- 专注于三个核心指令的批量操作
- 仅使用标准CAN，不依赖CANFD，确保最大兼容性
- 支持批量获取电机位置、批量发送位置命令、批量读取触觉数据
- 默认支持3个电机，可扩展到更多

支持的指令:
0xA0 - 读取电机信息 (返回电机信息结构体)
0xAA - 位置速度电流混合控制 (发送目标位置、速度和电流，返回电机信息结构体)
0xB6 - 读触觉数据 (发送帧号，返回帧号和6个触觉值)

批量操作方法:
1. 批量读取电机信息: controller.batch_get_motors_info([1,2,3])
2. 批量移动电机: controller.batch_move_motors([pos1,pos2,pos3], [speed1,speed2,speed3], [current1,current2,current3], [1,2,3])
3. 批量读取触觉数据: controller.batch_get_tactile_data([1,2,3], [frame1,frame2,frame3])
"""

import can
import time
import struct
from typing import Dict, List, Tuple, Optional, Union


class RH2ControllerBatch:
    """
    睿研智能灵巧手RH2批量控制器类 - 标准CAN版本
    
    专注于三个核心指令的批量操作，使用标准CAN确保最大兼容性：
    - 0xA0: 读取电机信息
    - 0xAA: 位置速度电流混合控制  
    - 0xB6: 读触觉数据
    """
    
    # 指令常量定义
    COMMAND_READ_MOTOR_INFO = 0xA0        # 读取电机信息
    COMMAND_CTRL_MOTOR_POSITION = 0xAA    # 位置速度电流混合控制
    COMMAND_READ_TACTILE_DATA = 0xB6      # 读触觉数据
    
    def __init__(self, interface: str = 'pcan', channel: str = 'PCAN_USBBUS1', 
                 bitrate: int = 1000000, auto_connect: bool = True, 
                 motor_ids: List[int] = [1, 2, 3]):
        """
        初始化RH2批量控制器
        
        Args:
            interface: CAN接口类型 ('pcan', 'socketcan', etc.)
            channel: CAN通道名称
            bitrate: CAN总线波特率
            auto_connect: 是否自动连接CAN总线
            motor_ids: 电机ID列表，默认[1,2,3]
        """
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.connected = False
        
        # 支持的电机ID列表
        self.motor_ids = motor_ids
        self.response_ids = [0x100 + i for i in self.motor_ids]
        
        # 指令类型映射表
        self.command_names = {
            0xA0: "读取电机信息",
            0xAA: "位置速度电流混合控制", 
            0xB6: "读触觉数据"
        }
        
        if auto_connect:
            self.connect()
    
    def connect(self) -> bool:
        """
        连接CAN总线 - 仅使用标准CAN
        
        Returns:
            bool: 连接是否成功
        """
        try:
            # 仅使用标准CAN，确保最大兼容性
            bus_kwargs = {
                "interface": self.interface,
                "channel": self.channel,
                "bitrate": self.bitrate,
                "state": can.BusState.ACTIVE,
            }
            
            self.bus = can.Bus(**bus_kwargs)
            self.connected = True
            print(f"CAN总线连接成功: {self.interface}:{self.channel} (标准CAN)")
            return True
        except Exception as e:
            print(f"CAN总线连接失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开CAN总线连接"""
        if self.bus and self.connected:
            self.bus.shutdown()
            self.connected = False
            print("CAN总线已断开")
    
    def is_connected(self) -> bool:
        """检查CAN总线是否已连接"""
        return self.connected
    
    def _validate_motor_ids(self, motor_ids: List[int]) -> bool:
        """验证电机ID列表"""
        for motor_id in motor_ids:
            if motor_id not in self.motor_ids:
                print(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
                return False
        return True
    
    def _send_command(self, command: int, motor_id: int, data_payload: List[int] = None) -> bool:
        """
        发送单个命令到指定电机
        
        Args:
            command: 指令代码 (0xA0, 0xAA, 0xB6)
            motor_id: 电机ID
            data_payload: 附加数据，如果为None则填充0
            
        Returns:
            bool: 发送是否成功
        """
        if not self.connected:
            print("CAN总线未连接")
            return False
        
        # 构造8字节数据包
        data = [command]
        if data_payload:
            data.extend(data_payload[:7])  # 最多7字节附加数据
        
        # 填充到8字节
        while len(data) < 8:
            data.append(0x00)
        
        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"发送失败 (电机{motor_id}): {e}")
            return False
    
    def _collect_responses(self, expected_motor_ids: List[int], timeout: float = 1.0) -> Dict[int, Optional[can.Message]]:
        """
        收集多个电机的应答
        
        Args:
            expected_motor_ids: 期望应答的电机ID列表
            timeout: 超时时间
            
        Returns:
            Dict[int, Optional[can.Message]]: {电机ID: 应答消息}
        """
        results = {}
        expected_response_ids = {0x100 + motor_id: motor_id for motor_id in expected_motor_ids}
        
        start_time = time.time()
        while time.time() - start_time < timeout and len(results) < len(expected_motor_ids):
            rx = self.bus.recv(timeout=0.1)
            if rx is None:
                continue
            
            motor_id = expected_response_ids.get(rx.arbitration_id)
            if motor_id is not None and motor_id not in results:
                results[motor_id] = rx
        
        # 对未收到应答的电机填充None
        for motor_id in expected_motor_ids:
            if motor_id not in results:
                results[motor_id] = None
        
        return results
    
    def _parse_motor_info_bitfields(self, data: bytes) -> Dict:
        """
        根据C结构体定义解析电机信息位字段
        
        C结构体格式 (MFingerInfo_t):
        uint64_t :8;          // CMD (已跳过)
        uint64_t status:8;    // 故障状态
        uint64_t P:12;        // 当前位置，0-4095 对应 0到满行程  
        uint64_t V:12;        // 当前速度，-2048~2047 单位 0.001行程/s
        uint64_t I:12;        // 当前电流，-2048~2047 单位 0.001A
        uint64_t F:12;        // 压力传感器ADC原始值，0-4095
        
        Args:
            data: 8字节原始数据
            
        Returns:
            Dict: 解析后的字段
        """
        # 将8字节数据转换为64位整数 (小端格式)
        raw_uint64 = struct.unpack('<Q', data)[0]
        
        # 按位字段解析
        # 跳过前8位 (CMD)
        raw_uint64 >>= 8
        
        # 提取status字段 (8位)
        status = raw_uint64 & 0xFF
        raw_uint64 >>= 8
        
        # 提取P字段 (12位位置)
        position = raw_uint64 & 0xFFF
        raw_uint64 >>= 12
        
        # 提取V字段 (12位速度，有符号)
        velocity_raw = raw_uint64 & 0xFFF
        # 转换为有符号数 (-2048~2047)
        if velocity_raw & 0x800:  # 检查符号位
            velocity = velocity_raw - 0x1000
        else:
            velocity = velocity_raw
        raw_uint64 >>= 12
        
        # 提取I字段 (12位电流，有符号)
        current_raw = raw_uint64 & 0xFFF
        # 转换为有符号数 (-2048~2047)
        if current_raw & 0x800:  # 检查符号位
            current = current_raw - 0x1000
        else:
            current = current_raw
        raw_uint64 >>= 12
        
        # 提取F字段 (12位压力传感器)
        force_sensor = raw_uint64 & 0xFFF
        
        return {
            'status': status,
            'position': position,
            'velocity': velocity,
            'current': current,
            'force_sensor': force_sensor
        }
    
    def _get_status_description(self, status_code: int) -> str:
        """
        获取状态码描述
        
        Args:
            status_code: 状态码
            
        Returns:
            str: 状态描述
        """
        status_descriptions = {
            0: "电机正常",
            1: "电机过温告警", 
            2: "电机过温保护",
            3: "电机低压保护",
            4: "电机过压保护",
            5: "电机过流保护",
            6: "电机力矩保护",
            7: "电机熔丝位错保护",
            8: "电机堵转保护"
        }
        
        return status_descriptions.get(status_code, f"未知状态码({status_code})")

    def _decode_motor_info_response(self, msg: can.Message, motor_id: int) -> Dict:
        """
        解码电机信息应答 (0xA0/0xAA) - 使用正确的位字段解析
        
        Args:
            msg: CAN消息
            motor_id: 电机ID
            
        Returns:
            Dict: 解码后的电机信息
        """
        if msg is None or len(msg.data) < 8:
            return {'error': f"电机{motor_id}未收到应答或数据长度不足"}
        
        data = msg.data
        
        # 基本信息
        decoded = {
            'motor_id': motor_id,
            'command_type': hex(data[0]),
            'command_name': self.command_names.get(data[0], f"未知指令({hex(data[0])})"),
            'timestamp': time.time(),
            'raw_data': [hex(x) for x in data],
            'raw_bytes': list(data)
        }
        
        try:
            if data[0] in [self.COMMAND_READ_MOTOR_INFO, self.COMMAND_CTRL_MOTOR_POSITION]:
                # 使用位字段解析
                parsed_fields = self._parse_motor_info_bitfields(data)
                
                # 计算物理量
                position = parsed_fields['position']  # 0-4095 对应满行程
                velocity_raw = parsed_fields['velocity']  # -2048~2047, 单位 0.001行程/s
                current_raw = parsed_fields['current']   # -2048~2047, 单位 0.001A
                force_sensor = parsed_fields['force_sensor']  # 0-4095 ADC原始值
                status = parsed_fields['status']
                
                # 转换为物理单位
                velocity_physical = velocity_raw * 0.001  # 行程/s
                current_physical = current_raw * 0.001    # A (安培)
                current_ma = current_raw                  # mA (毫安)
                
                decoded.update({
                    # 原始值
                    'status_code': status,
                    'position_raw': position,
                    'velocity_raw': velocity_raw,
                    'current_raw': current_raw,
                    'force_sensor_raw': force_sensor,
                    
                    # 物理量 (兼容原接口)
                    'current_position': position,
                    'current_speed': velocity_raw,  # 保持原始单位便于兼容
                    'current_current': current_ma,  # mA单位
                    'status': status,
                    
                    # 新的物理量字段
                    'position_normalized': position / 4095.0,  # 0.0-1.0 归一化位置
                    'velocity_physical': velocity_physical,     # 行程/s
                    'current_physical': current_physical,       # A
                    'current_ma': current_ma,                   # mA
                    'force_sensor_adc': force_sensor,          # ADC原始值
                    'force_sensor_normalized': force_sensor / 4095.0,  # 0.0-1.0
                    
                    # 状态描述
                    'status_description': self._get_status_description(status),
                    'is_normal': status == 0,
                    'has_error': status != 0,
                    
                    # 单位说明
                    'units': {
                        'position': '0-4095 (满行程)',
                        'velocity': '0.001行程/s',
                        'current': 'mA',
                        'force_sensor': 'ADC原始值 (0-4095)'
                    }
                })
                
                # 添加C结构体解析演示
                decoded['c_struct_demo'] = {
                    'description': '对应C结构体MFingerInfo_t的字段',
                    'P': position,      # 当前位置
                    'V': velocity_raw,  # 当前速度
                    'I': current_raw,   # 当前电流  
                    'F': force_sensor,  # 压力传感器
                    'status': status    # 故障状态
                }
                
            else:
                decoded['error'] = f"意外的指令类型: 0x{data[0]:02X}"
                
        except Exception as e:
            decoded['error'] = f"解码过程中发生错误: {e}"
            decoded['exception_details'] = str(e)
        
        return decoded
    
    def _decode_tactile_response(self, msg: can.Message, motor_id: int) -> Dict:
        """
        解码触觉数据应答 (0xB6)
        
        Args:
            msg: CAN消息
            motor_id: 电机ID
            
        Returns:
            Dict: 解码后的触觉数据
        """
        if msg is None or len(msg.data) < 8:
            return {'error': f"电机{motor_id}未收到触觉数据应答或数据长度不足"}
        
        data = msg.data
        
        decoded = {
            'motor_id': motor_id,
            'command_type': hex(data[0]),
            'command_name': self.command_names.get(data[0], f"未知指令({hex(data[0])})"),
            'timestamp': time.time(),
            'raw_data': [hex(x) for x in data]
        }
        
        try:
            if data[0] == self.COMMAND_READ_TACTILE_DATA:
                frame_number = data[1]
                tactile_values = list(data[2:8])  # 6个触觉值
                
                decoded.update({
                    'frame_number': frame_number,
                    'tactile_values': tactile_values,
                    'tactile_units': '触觉单位'
                })
            else:
                decoded['error'] = f"意外的指令类型: 0x{data[0]:02X}"
                
        except Exception as e:
            decoded['error'] = f"解码触觉数据时发生错误: {e}"
        
        return decoded
    
    # 批量操作方法
    
    def batch_get_motors_info(self, motor_ids: List[int] = None, timeout: float = 1.0) -> Dict[int, Dict]:
        """
        批量获取电机信息 (0xA0)
        
        Args:
            motor_ids: 电机ID列表，如果为None则使用默认的motor_ids
            timeout: 超时时间
            
        Returns:
            Dict[int, Dict]: {电机ID: 电机信息}
        """
        if motor_ids is None:
            motor_ids = self.motor_ids
        
        if not self._validate_motor_ids(motor_ids):
            return {}
        
        print(f"批量获取电机信息: {motor_ids}")
        
        # 批量发送0xA0指令
        success_count = 0
        for motor_id in motor_ids:
            if self._send_command(self.COMMAND_READ_MOTOR_INFO, motor_id):
                success_count += 1
        
        if success_count == 0:
            print("所有指令发送失败")
            return {}
        
        # 收集应答
        responses = self._collect_responses(motor_ids, timeout)
        
        # 解码结果
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._decode_motor_info_response(msg, motor_id)
        
        return results
    
    def batch_move_motors(self, positions: List[int], speeds: List[int], 
                         current_limits: List[int], motor_ids: List[int] = None, 
                         timeout: float = 1.0) -> Dict[int, Dict]:
        """
        批量移动电机到指定位置 (0xAA)
        
        Args:
            positions: 目标位置列表
            speeds: 速度列表  
            current_limits: 电流限制列表
            motor_ids: 电机ID列表，如果为None则使用默认的motor_ids
            timeout: 超时时间
            
        Returns:
            Dict[int, Dict]: {电机ID: 电机应答信息}
        """
        if motor_ids is None:
            motor_ids = self.motor_ids
        
        if not self._validate_motor_ids(motor_ids):
            return {}
        
        # 验证参数长度
        if not (len(positions) == len(speeds) == len(current_limits) == len(motor_ids)):
            print("参数列表长度不匹配")
            return {}
        
        print(f"批量移动电机: {motor_ids}")
        
        # 批量发送0xAA指令
        success_count = 0
        for i, motor_id in enumerate(motor_ids):
            # 构造位置控制数据包
            payload = [
                positions[i] & 0xFF,           # 目标位置低字节
                (positions[i] >> 8) & 0xFF,   # 目标位置高字节
                speeds[i] & 0xFF,             # 速度低字节
                (speeds[i] >> 8) & 0xFF,      # 速度高字节
                current_limits[i] & 0xFF,     # 电流低字节
                (current_limits[i] >> 8) & 0xFF, # 电流高字节
                0x00                          # 填充字节
            ]
            
            if self._send_command(self.COMMAND_CTRL_MOTOR_POSITION, motor_id, payload):
                success_count += 1
                print(f"  电机{motor_id}: 位置={positions[i]}, 速度={speeds[i]}, 电流={current_limits[i]}")
        
        if success_count == 0:
            print("所有位置指令发送失败")
            return {}
        
        # 收集应答
        responses = self._collect_responses(motor_ids, timeout)
        
        # 解码结果
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._decode_motor_info_response(msg, motor_id)
        
        return results
    
    def batch_get_tactile_data(self, motor_ids: List[int] = None, 
                              frame_numbers: List[int] = None, 
                              timeout: float = 1.0) -> Dict[int, Dict]:
        """
        批量获取触觉数据 (0xB6)
        
        Args:
            motor_ids: 电机ID列表，如果为None则使用默认的motor_ids
            frame_numbers: 帧号列表，如果为None则全部使用0
            timeout: 超时时间
            
        Returns:
            Dict[int, Dict]: {电机ID: 触觉数据}
        """
        if motor_ids is None:
            motor_ids = self.motor_ids
        
        if frame_numbers is None:
            frame_numbers = [0] * len(motor_ids)
        
        if not self._validate_motor_ids(motor_ids):
            return {}
        
        if len(frame_numbers) != len(motor_ids):
            print("帧号列表长度与电机ID列表长度不匹配")
            return {}
        
        print(f"批量获取触觉数据: {motor_ids}")
        
        # 批量发送0xB6指令
        success_count = 0
        for i, motor_id in enumerate(motor_ids):
            payload = [frame_numbers[i]]  # 帧号
            if self._send_command(self.COMMAND_READ_TACTILE_DATA, motor_id, payload):
                success_count += 1
                print(f"  电机{motor_id}: 帧号={frame_numbers[i]}")
        
        if success_count == 0:
            print("所有触觉数据指令发送失败")
            return {}
        
        # 收集应答
        responses = self._collect_responses(motor_ids, timeout)
        
        # 解码结果
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._decode_tactile_response(msg, motor_id)
        
        return results
    
    # 便捷方法
    
    def get_all_positions(self) -> Dict[int, int]:
        """
        获取所有电机的当前位置
        
        Returns:
            Dict[int, int]: {电机ID: 位置}
        """
        infos = self.batch_get_motors_info()
        positions = {}
        for motor_id, info in infos.items():
            if 'current_position' in info:
                positions[motor_id] = info['current_position']
            else:
                positions[motor_id] = None
        return positions
    
    def move_all_motors_to_same_position(self, position: int, speed: int = 3000, 
                                       current_limit: int = 1000) -> Dict[int, Dict]:
        """
        移动所有电机到相同位置
        
        Args:
            position: 目标位置
            speed: 速度
            current_limit: 电流限制
            
        Returns:
            Dict[int, Dict]: 执行结果
        """
        positions = [position] * len(self.motor_ids)
        speeds = [speed] * len(self.motor_ids)
        current_limits = [current_limit] * len(self.motor_ids)
        
        return self.batch_move_motors(positions, speeds, current_limits)
    
    def get_all_tactile_data(self, frame_number: int = 0) -> Dict[int, Dict]:
        """
        获取所有电机的触觉数据
        
        Args:
            frame_number: 帧号
            
        Returns:
            Dict[int, Dict]: 触觉数据
        """
        frame_numbers = [frame_number] * len(self.motor_ids)
        return self.batch_get_tactile_data(frame_numbers=frame_numbers)
    
    # 上下文管理器支持
    
    def __enter__(self):
        """上下文管理器入口"""
        if not self.connected:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()
    
    def __del__(self):
        """析构函数"""
        self.disconnect()


def test_batch_controller():
    """测试批量控制器功能"""
    print("=== RH2批量控制器测试 (标准CAN) ===")
    
    # 使用上下文管理器自动管理连接
    with RH2ControllerBatch(motor_ids=[2, 3, 4, 5, 6]) as controller:
        if not controller.is_connected():
            print("连接失败，退出测试")
            return
        
        print("连接成功，开始测试...")
        
        # 测试1: 批量获取电机信息
        print("\n1. 批量获取电机信息 (0xA0):")
        motor_infos = controller.batch_get_motors_info()
        for motor_id, info in motor_infos.items():
            if 'error' not in info:
                print(f"  电机{motor_id}: 位置={info.get('current_position')}, "
                      f"速度={info.get('current_speed')}, 电流={info.get('current_current')}, "
                      f"状态={info.get('status')}")
            else:
                print(f"  电机{motor_id}: {info['error']}")
        
        time.sleep(1)
        
        # 测试2: 移动所有电机到4096位置并验证
        print("\n2. 移动所有电机到4096位置:")
        target_position = 4096
        print(f"   发送移动指令: 目标位置={target_position}")
        move_results = controller.move_all_motors_to_same_position(target_position, 2000, 800)
        
        for motor_id, result in move_results.items():
            if 'error' not in result:
                print(f"   电机{motor_id}: 移动指令已发送，应答状态={result.get('status')}")
            else:
                print(f"   电机{motor_id}: 发送失败 - {result['error']}")
        
        # 等待电机运动完成
        print("   等待电机运动完成...")
        time.sleep(3)
        
        # 测试3: 读取运动后的位置，验证是否接近4096
        print(f"\n3. 验证电机位置是否接近目标位置{target_position}:")
        positions_after_move = controller.get_all_positions()
        
        for motor_id, position in positions_after_move.items():
            if position is not None:
                position_error = abs(position - target_position)
                error_percentage = (position_error / target_position) * 100
                
                print(f"   电机{motor_id}: 当前位置={position}, 目标位置={target_position}")
                print(f"   电机{motor_id}: 位置误差={position_error}, 误差百分比={error_percentage:.2f}%")
                
                # 判断位置是否在合理范围内（允许5%误差）
                if position_error <= target_position * 0.05:
                    print(f"   电机{motor_id}: ✅ 位置正确")
                else:
                    print(f"   电机{motor_id}: ❌ 位置误差过大")
            else:
                print(f"   电机{motor_id}: ❌ 无法读取位置")
        
        time.sleep(1)
        
        # 测试4: 批量获取详细电机信息
        print("\n4. 获取详细电机信息 (使用C结构体解析):")
        detailed_infos = controller.batch_get_motors_info()
        for motor_id, info in detailed_infos.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                speed = info.get('current_speed', 'N/A')
                current = info.get('current_current', 'N/A')
                status = info.get('status', 'N/A')
                status_desc = info.get('status_description', 'N/A')
                
                print(f"   电机{motor_id}: 位置={pos}, 速度={speed}, 电流={current}mA")
                print(f"   电机{motor_id}: 状态=0x{status:02X} ({status_desc})")
                
                # 显示C结构体字段
                c_struct = info.get('c_struct_demo', {})
                if c_struct:
                    print(f"   电机{motor_id}: C结构体 P={c_struct.get('P')}, V={c_struct.get('V')}, I={c_struct.get('I')}, F={c_struct.get('F')}")
                
                # 显示物理量
                force_adc = info.get('force_sensor_adc', 'N/A')
                pos_norm = info.get('position_normalized', 0)
                print(f"   电机{motor_id}: 位置百分比={pos_norm*100:.1f}%, 压力传感器ADC={force_adc}")
                
            else:
                print(f"   电机{motor_id}: {info['error']}")
        
        time.sleep(1)
        
        # 测试5: 回零所有电机
        print("\n5. 回零所有电机:")
        zero_results = controller.move_all_motors_to_same_position(0, 2000, 800)
        for motor_id, result in zero_results.items():
            if 'error' not in result:
                print(f"   电机{motor_id}: 回零指令已发送")
            else:
                print(f"   电机{motor_id}: {result['error']}")
        
        # 等待回零完成
        time.sleep(2)
        
        # 验证回零位置
        print("\n6. 验证回零位置:")
        zero_positions = controller.get_all_positions()
        for motor_id, position in zero_positions.items():
            if position is not None:
                print(f"   电机{motor_id}: 回零后位置={position}")
                if position <= 100:  # 允许小范围误差
                    print(f"   电机{motor_id}: ✅ 回零成功")
                else:
                    print(f"   电机{motor_id}: ❌ 回零不完全")
            else:
                print(f"   电机{motor_id}: ❌ 无法读取回零位置")


if __name__ == "__main__":
    test_batch_controller()
