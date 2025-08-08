#!/usr/bin/env python3
"""
睿研智能灵巧手 RH2 控制器库 - 精简版本
硬件  : PEAK PCAN-USB
驱动  : peak-linux-driver
库    : python-can           pip install python-can
协议  : 根据睿研智能灵巧手通信协议.csv

多电机支持:
- 每个模块最多8个独立ID（设备ID范围1~254，0为广播，255无效）
- 设备应答ID为 本身ID + 256（即 0x100 + 设备ID）
- 支持单独控制与多电机（FD）批量

支持的指令:
0xA0 - 读取电机信息 (返回电机信息结构体)
0xAA - 位置速度电流混合控制 (发送目标位置、速度和电流，返回电机信息结构体)
0xB6 - 读触觉数据 (发送帧号，返回帧号和6个触觉值)

使用方法:
1. 读取电机信息: controller.get_motor_info(motor_id)
2. 移动电机: controller.move_motor_to_position(motor_id, position, speed, current_limit)
3. 读取触觉数据: controller.get_tactile_data(motor_id, frame_number)
"""

import can
import time
import struct
from typing import Dict, List, Tuple, Optional, Union


class RH2Controller:
    """
    睿研智能灵巧手RH2控制器类 - 精简版本
    
    提供三个核心指令的电机控制接口，支持6个电机的单独控制：
    - 0xA0: 读取电机信息
    - 0xAA: 位置速度电流混合控制  
    - 0xB6: 读触觉数据
    """
    
    # 指令常量定义 - 精简版本
    COMMAND_READ_MOTOR_INFO = 0xA0        # 读取电机信息
    COMMAND_CTRL_MOTOR_POSITION = 0xAA    # 位置速度电流混合控制
    COMMAND_READ_TACTILE_DATA = 0xB6      # 读触觉数据
    
    
    def __init__(self, interface: str = 'pcan', channel: str = 'PCAN_USBBUS1', 
                 bitrate: int = 1000000, auto_connect: bool = True, motor_ids:[int] = [1, 2, 3, 4, 5, 6],
                 use_fd: bool = False, data_bitrate: Optional[int] = None):
        """
        初始化RH2控制器
        
        Args:
            interface: CAN接口类型 ('pcan', 'socketcan', etc.)
            channel: CAN通道名称
            bitrate: CAN总线波特率
            auto_connect: 是否自动连接CAN总线
        """
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.connected = False
        
        # 支持多个电机，ID由motor_ids参数指定
        self.motor_ids = motor_ids
        self.response_ids = [0x100 + i for i in self.motor_ids]

        # CAN FD 配置（依据协议：CANFD 1M + 5M）。注意：具体后端驱动的FD参数设置可能不同，这里仅保留标志与切换位。
        self.use_fd = use_fd
        self.data_bitrate = data_bitrate
        
        # 指令类型映射表 - 精简版本
        self.command_names = {
            0xA0: "读取电机信息",
            0xAA: "位置速度电流混合控制", 
            0xB6: "读触觉数据"
        }
        
        if auto_connect:
            self.connect()
    
    def connect(self) -> bool:
        """
        连接CAN总线
        
        Returns:
            bool: 连接是否成功
        """
        try:
            # 检查设备是否支持 FD
            fd_supported = False
            if self.use_fd:
                try:
                    configs = can.detect_available_configs()
                    for config in configs:
                        if (config.get('interface') == self.interface and 
                            config.get('channel') == self.channel):
                            fd_supported = config.get('supports_fd', False)
                            break
                except Exception:
                    fd_supported = False
                
                if not fd_supported:
                    print(f"设备 {self.channel} 不支持 FD，回退到普通 CAN 模式")
                    self.use_fd = False

            # 构建连接参数
            bus_kwargs = {
                "interface": self.interface,
                "channel": self.channel,
                "bitrate": self.bitrate,
                "state": can.BusState.ACTIVE,
            }
            
            # 只有在设备支持 FD 时才添加 FD 参数
            if self.use_fd and fd_supported:
                bus_kwargs["fd"] = True
                bus_kwargs["data_bitrate"] = self.data_bitrate or 5_000_000
                print(f"启用 CAN FD 模式，数据速率: {bus_kwargs['data_bitrate']} bps")
            
            self.bus = can.Bus(**bus_kwargs)
            self.connected = True
            mode = "FD" if (self.use_fd and fd_supported) else "Classic"
            print(f"CAN总线连接成功: {self.interface}:{self.channel} ({mode})")
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
    

    
    def send_motor_position_command(self, target_position: int, speed: int, 
                                  current_limit: int, motor_id: int) -> bool:
        """
        发送位置速度电流混合控制指令 (0xAA)
        
        Args:
            target_position: 目标位置 (0-4095)
            speed: 到达目标位置速度
            current_limit: 过程最大电流值
            motor_id: 电机ID（1-254，以 motor_ids 列表为准），必需参数
            
        Returns:
            bool: 发送是否成功
        """
        if not self.connected:
            print("CAN总线未连接")
            return False
            
        # 验证电机ID
        if motor_id not in self.motor_ids:
            print(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
            return False
        
        # 根据协议：0xAA + 电机1目标位置(2字节) + 电机1到达目标位置速度(2字节) + 电机1过程最大电流值(2字节)
        data = [
            self.COMMAND_CTRL_MOTOR_POSITION,  # 位置速度电流混合控制
            target_position & 0xFF,  # 目标位置低字节
            (target_position >> 8) & 0xFF,  # 目标位置高字节
            speed & 0xFF,  # 速度低字节
            (speed >> 8) & 0xFF,  # 速度高字节
            current_limit & 0xFF,  # 电流低字节
            (current_limit >> 8) & 0xFF,  # 电流高字节
            0x00   # 填充字节
        ]
        
        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"发送失败: {e}")
            return False
    
    def send_read_command(self, command: int, motor_id: int) -> bool:
        """
        发送读取指令
        
        Args:
            command: 读取指令 (0xA0, 0xB6)
            motor_id: 电机ID（1-254，以 motor_ids 列表为准），必需参数
            
        Returns:
            bool: 发送是否成功
        """
        if not self.connected:
            print("CAN总线未连接")
            return False
            
        # 验证电机ID
        if motor_id not in self.motor_ids:
            print(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
            return False
        
        data = [command] + [0x00] * 7  # 填充剩余字节为0
        
        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"发送失败: {e}")
            return False
    
    def send_tactile_data_command(self, frame_number: int, motor_id: int) -> bool:
        """
        发送读触觉数据指令 (0xB6)
        
        Args:
            frame_number: 帧号(n)
            motor_id: 电机ID（1-254，以 motor_ids 列表为准），必需参数
            
        Returns:
            bool: 发送是否成功
        """
        if not self.connected:
            print("CAN总线未连接")
            return False
            
        # 验证电机ID
        if motor_id not in self.motor_ids:
            print(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
            return False
        
        # 根据协议：0xB6 + 帧号(n)
        data = [
            self.COMMAND_READ_TACTILE_DATA,  # 读触觉数据
            frame_number,  # 帧号
            0x00,  # 填充字节
            0x00,  # 填充字节
            0x00,  # 填充字节
            0x00,  # 填充字节
            0x00,  # 填充字节
            0x00   # 填充字节
        ]
        
        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"发送失败: {e}")
            return False
    
    def read_motor_status(self, motor_id: int, timeout: float = 1.0) -> Optional[Dict]:
        """
        读取电机状态信息
        
        Args:
            timeout: 超时时间(秒)
            motor_id: 电机ID（1-254，以 motor_ids 列表为准），必需参数
            
        Returns:
            Dict: 解码后的状态信息，如果失败返回None
        """
        if not self.connected:
            print("CAN总线未连接")
            return None
            
        # 验证电机ID
        if motor_id not in self.motor_ids:
            print(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
            return None
        
        response_id = 0x100 + motor_id  # 计算对应的回复ID
        start_time = time.time()
        while time.time() - start_time < timeout:
            rx = self.bus.recv(timeout=0.1)
            if rx is None:
                continue
                
            if rx.arbitration_id == response_id:
                return self.decode_status_response(rx, motor_id, response_id)
        
        return None
    
    def decode_status_response(self, msg: can.Message, motor_id: int, response_id: int) -> Dict:
        """
        解码状态应答数据
        
        Args:
            msg: CAN消息
            motor_id: 电机ID
            response_id: 回复ID
            
        Returns:
            Dict: 解码后的数据字典
        """
        if len(msg.data) < 8:
            return {'error': f"数据长度不足: {len(msg.data)}"}
            
        data = msg.data
        
        # 根据协议解码数据
        decoded = {
            'raw_data': [hex(x) for x in data],
            'raw_len': len(data),
            'command_type': hex(data[0]),
            'command_name': self.command_names.get(data[0], f"未知指令({hex(data[0])})"),
            'timestamp': time.time(),
            'motor_id': motor_id,
            'response_id': response_id
        }
        
        # 根据不同的指令类型解码
        try:
            if data[0] == self.COMMAND_READ_MOTOR_INFO:  # 读取电机信息
                # 0xA0返回电机信息结构体（根据实际测试确认的字段布局）
                # 正确映射：pos = [1:2]LE, speed = [3:4]LE, current = [5:6]LE, status = [7]
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
                # 兼容FD返回每电机16字节的扩展场
                if len(data) >= 16:
                    decoded['extra_raw'] = [hex(x) for x in data[8:16]]
                
            elif data[0] == self.COMMAND_CTRL_MOTOR_POSITION:  # 位置速度电流混合控制
                # 0xAA返回电机信息结构体（与A0一致的布局）
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
                if len(data) >= 16:
                    decoded['extra_raw'] = [hex(x) for x in data[8:16]]
                
            elif data[0] == self.COMMAND_READ_TACTILE_DATA:  # 读触觉数据
                # 根据协议文档，0xB6返回触觉数据
                frame_number = data[1]
                tactile_values = data[2:8]  # 6个触觉值
                
                decoded.update({
                    'frame_number': frame_number,
                    'tactile_values': tactile_values,
                    'tactile_units': '触觉单位'
                })
                
            else:
                decoded['error'] = f"未知指令类型: 0x{data[0]:02X}"
                
        except Exception as e:
            decoded['error'] = f"解码过程中发生错误: {e}"
            
        return decoded
    
    def request_all_motors_info_once(self, overall_timeout: float = 1.0) -> Dict[int, Optional[Dict]]:
        """
        使用0xA0一次性请求所有电机信息，并在一个收包窗口内收集并解码所有电机的应答。

        按照协议：电机ID为n(1-6)，其应答ID为0x100 + n，帧首字节为指令码(此处期望为0xA0)。

        Args:
            overall_timeout: 整体收包窗口(秒)。在该时间内尽可能收齐所有电机的A0应答。

        Returns:
            Dict[int, Optional[Dict]]: {电机ID: 解码结果或None}
        """
        results: Dict[int, Optional[Dict]] = {}

        if not self.connected:
            print("CAN总线未连接")
            for motor_id in self.motor_ids:
                results[motor_id] = None
            return results

        # 优先尝试“多电机控制指令”FD批量请求：
        # 依据协议：长度为 8 * 电机数。发送帧：byte0 为 0xA0，其后每电机7字节控制数据（此处都置0）。
        # 发送ID：以第1个电机的ID进行发送。
        fd_sent = False
        if self.use_fd and len(self.motor_ids) > 0:
            try:
                num_motors = len(self.motor_ids)
                total_len = 8 * num_motors
                payload = bytearray(total_len)
                # 第一个电机块的 byte0 放指令 0xA0，其余控制参数置0
                for idx in range(num_motors):
                    base = 8 * idx
                    payload[base + 0] = self.COMMAND_READ_MOTOR_INFO
                    # base + 1..7 全为 0（已是默认）

                fd_msg = can.Message(
                    arbitration_id=self.motor_ids[0],
                    data=payload,
                    is_extended_id=False,
                    is_fd=True,
                    bitrate_switch=True
                )
                self.bus.send(fd_msg)
                fd_sent = True
            except Exception:
                fd_sent = False

        if not fd_sent:
            # 回退到逐电机单独请求
            for motor_id in self.motor_ids:
                self.send_read_command(self.COMMAND_READ_MOTOR_INFO, motor_id)

        # 构建期望的应答ID映射：{arbitration_id: motor_id}
        expected_resp_id_to_motor: Dict[int, int] = {0x100 + mid: mid for mid in self.motor_ids}

        deadline = time.time() + overall_timeout
        while time.time() < deadline and len(results) < len(self.motor_ids):
            rx = self.bus.recv(timeout=0.05)
            if rx is None:
                continue

            if rx.is_fd and fd_sent:
                # FD 批量返回：长度应为 8 * 电机数。按 8 字节分块解析，每块的第0字节为返回码，预期 0xA0。
                data_bytes = bytes(rx.data)
                # 推断每电机块大小（8 或 16），默认 8
                block_size = 8
                if len(self.motor_ids) > 0 and len(data_bytes) % len(self.motor_ids) == 0:
                    candidate = len(data_bytes) // len(self.motor_ids)
                    if candidate in (8, 16):
                        block_size = candidate
                if len(data_bytes) % block_size == 0:
                    num_blocks = len(data_bytes) // block_size
                    for idx in range(min(num_blocks, len(self.motor_ids))):
                        base = block_size * idx
                        block = data_bytes[base:base+block_size]
                        if self.motor_ids[idx] in results:
                            continue
                        short8 = block[:8]
                        if short8[0] == self.COMMAND_READ_MOTOR_INFO:
                            msg_data = block if block_size >= 8 else short8
                        else:
                            # 兼容块首不带返回码，补上 A0
                            msg_data = bytes([self.COMMAND_READ_MOTOR_INFO]) + short8[0:7]
                            if block_size > 8:
                                # 保留扩展字段
                                msg_data += block[8:block_size]
                        pseudo_msg = can.Message(
                            arbitration_id=0x100 + self.motor_ids[idx],
                            data=msg_data,
                            is_extended_id=False,
                            is_fd=True,
                            bitrate_switch=True
                        )
                        results[self.motor_ids[idx]] = self.decode_status_response(
                            pseudo_msg, self.motor_ids[idx], 0x100 + self.motor_ids[idx]
                        )
                # 若收到了 FD 返回，无需继续在此帧上做普通ID匹配，继续收下一帧或退出
                continue
            else:
                # 非 FD 或 FD 发送失败情况下，按照单电机应答 ID 处理
                motor_id = expected_resp_id_to_motor.get(rx.arbitration_id)
                if motor_id is None:
                    continue
                if len(rx.data) >= 1 and rx.data[0] == self.COMMAND_READ_MOTOR_INFO:
                    if motor_id not in results:
                        results[motor_id] = self.decode_status_response(rx, motor_id, rx.arbitration_id)
                else:
                    if motor_id not in results:
                        results[motor_id] = None

        # 对未收到的电机填None，便于上层统一处理
        for motor_id in self.motor_ids:
            if motor_id not in results:
                results[motor_id] = None

        return results

    def get_motor_info(self, motor_id: int) -> Optional[Dict]:
        """
        获取指定电机的完整信息 (0xA0)
        
        Args:
            motor_id: 电机ID（1-254，以 motor_ids 列表为准）
            
        Returns:
            Optional[Dict]: 电机信息，失败返回None
        """
        if self.send_read_command(self.COMMAND_READ_MOTOR_INFO, motor_id):
            status = self.read_motor_status(motor_id)
            return status
        return None
    
    def get_tactile_data(self, motor_id: int, frame_number: int = 0) -> Optional[Dict]:
        """
        获取指定电机的触觉数据 (0xB6)
        
        Args:
            motor_id: 电机ID (1-6)
            frame_number: 帧号
            
        Returns:
            Optional[Dict]: 触觉数据，失败返回None
        """
        if self.send_tactile_data_command(frame_number, motor_id):
            status = self.read_motor_status(motor_id)
            if status and 'tactile_values' in status:
                return status
        return None
    
    def move_motor_to_position(self, motor_id: int, target_position: int, 
                             speed: int = 5000, current_limit: int = 3000) -> bool:
        """
        移动指定电机到目标位置 (0xAA)
        
        Args:
            motor_id: 电机ID (1-6)
            target_position: 目标位置 (0-4095)
            speed: 到达目标位置速度
            current_limit: 过程最大电流值
            
        Returns:
            bool: 发送是否成功
        """
        return self.send_motor_position_command(target_position, speed, current_limit, motor_id)
    
    def get_all_motors_info(self) -> Dict[int, Optional[Dict]]:
        """
        获取所有电机的信息（批量A0，一次性收集）
        
        Returns:
            Dict[int, Optional[Dict]]: 每个电机的信息 {电机ID: 信息}
        """
        return self.request_all_motors_info_once(overall_timeout=1.0)
    
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




def test_rh2_controller():
    """测试RH2控制器功能 - 精简版本"""
    print("=== RH2控制器测试 (精简版本) ===")
    
    # 使用上下文管理器自动管理连接
    with RH2Controller() as controller:
        if not controller.is_connected():
            print("连接失败，退出测试")
            return
        
        print("连接成功，开始测试...")
        
        # 测试1: 一次性请求并收集所有电机A0信息 (FD优先) 便于和协议CSV逐字段对比
        print("\n1. 一次性获取所有电机信息 (0xA0 批量, FD优先):")
        motor_infos_once = controller.request_all_motors_info_once(overall_timeout=1.0)
        for motor_id, info in motor_infos_once.items():
            if info:
                print(
                    f"  电机{motor_id}: id=0x{(0x100+motor_id):03X}, 指令={info.get('command_type')}, 原始={info.get('raw_data')}, "
                    f"位置={info.get('current_position', 'N/A')}, 速度={info.get('current_speed', 'N/A')}, 电流={info.get('current_current', 'N/A')}, 状态={info.get('status', 'N/A')}"
                )
            else:
                print(f"  电机{motor_id}: 未在窗口内收到0xA0应答或帧异常")

        time.sleep(2)
        # 测试2: 移动电机1到位置0 (0xAA)
        print("\n2. 移动电机1到位置4096 (0xAA):")
        success = controller.move_motor_to_position(controller.motor_ids[2], 4096, 3000, 800)
        success = controller.move_motor_to_position(controller.motor_ids[3], 4096, 3000, 800)
        success = controller.move_motor_to_position(controller.motor_ids[4], 4096, 3000, 800)
        success = controller.move_motor_to_position(controller.motor_ids[5], 4096, 3000, 800)
        time.sleep(2)


        # 测试1: 一次性请求并收集所有电机A0信息 (FD优先) 便于和协议CSV逐字段对比
        # print("\n1. 一次性获取所有电机信息 (0xA0 批量, FD优先):")
        # motor_infos_once = controller.request_all_motors_info_once(overall_timeout=1.0)
        # for motor_id, info in motor_infos_once.items():
        #     if info:
        #         print(
        #             f"  电机{motor_id}: id=0x{(0x100+motor_id):03X}, 指令={info.get('command_type')}, 原始={info.get('raw_data')}, "
        #             f"位置={info.get('current_position', 'N/A')}, 速度={info.get('current_speed', 'N/A')}, 电流={info.get('current_current', 'N/A')}, 状态={info.get('status', 'N/A')}"
        #         )
        #     else:
        #         print(f"  电机{motor_id}: 未在窗口内收到0xA0应答或帧异常")

        # time.sleep(2)


        success = controller.move_motor_to_position(controller.motor_ids[2], 0, 3000, 800)
        success = controller.move_motor_to_position(controller.motor_ids[3], 0, 3000, 800)
        success = controller.move_motor_to_position(controller.motor_ids[4], 0, 3000, 800)
        success = controller.move_motor_to_position(controller.motor_ids[5], 0, 3000, 800)

        print(f"  电机1: {'成功' if success else '失败'}")
        
        time.sleep(2)
        
        # # 测试3: 获取电机1的触觉数据 (0xB6)
        # print("\n3. 获取电机1的触觉数据 (0xB6):")
        # tactile_data = controller.get_tactile_data(1, 0)
        # if tactile_data:
        #     print(f"  电机1: 帧号={tactile_data.get('frame_number', 'N/A')}, "
        #           f"触觉值={tactile_data.get('tactile_values', 'N/A')}")
        # else:
        #     print(f"  电机1: 获取失败")
        
        # # 测试4: 再次获取电机1信息，查看位置变化
        # print("\n4. 再次获取电机1信息:")
        # motor_info = controller.get_motor_info(1)
        # if motor_info:
        #     print(f"  电机1: 位置={motor_info.get('mapped_position', 'N/A')}, "
        #           f"速度={motor_info.get('current_speed', 'N/A')}, "
        #           f"电流={motor_info.get('current_current', 'N/A')}")
        # else:
        #     print(f"  电机1: 获取失败")


if __name__ == "__main__":
    test_rh2_controller() 