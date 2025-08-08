#!/usr/bin/env python3

import can
import time
import struct
from typing import Dict, List, Optional


class RH2Controller:
    
    COMMAND_READ_MOTOR_INFO = 0xA0
    COMMAND_CTRL_MOTOR_POSITION = 0xAA
    COMMAND_READ_TACTILE_DATA = 0xB6
    
    def __init__(self, interface: str = 'pcan', channel: str = 'PCAN_USBBUS1', 
                 bitrate: int = 1000000, auto_connect: bool = True, 
                 motor_ids: List[int] = [1, 2, 3]):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.connected = False
        self.motor_ids = motor_ids
        self.response_ids = [0x100 + i for i in self.motor_ids]
        self.command_names = {
            0xA0: "读取电机信息",
            0xAA: "位置速度电流混合控制", 
            0xB6: "读触觉数据"
        }
        if auto_connect:
            self.connect()
    
    def connect(self) -> bool:
        try:
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
        if self.bus and self.connected:
            self.bus.shutdown()
            self.connected = False
            print("CAN总线已断开")
    
    def is_connected(self) -> bool:
        return self.connected
    
    def _validate_motor_ids(self, motor_ids: List[int]) -> bool:
        for motor_id in motor_ids:
            if motor_id not in self.motor_ids:
                print(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
                return False
        return True
    
    def _send_command(self, command: int, motor_id: int, data_payload: List[int] = None) -> bool:
        if not self.connected:
            print("CAN总线未连接")
            return False
        
        data = [command]
        if data_payload:
            data.extend(data_payload[:7])
        
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
        
        for motor_id in expected_motor_ids:
            if motor_id not in results:
                results[motor_id] = None
        
        return results
    
    def _parse_motor_info_bitfields(self, data: bytes) -> Dict:
        raw_uint64 = struct.unpack('<Q', data)[0]
        raw_uint64 >>= 8
        
        status = raw_uint64 & 0xFF
        raw_uint64 >>= 8
        
        position = raw_uint64 & 0xFFF
        raw_uint64 >>= 12
        
        velocity_raw = raw_uint64 & 0xFFF
        if velocity_raw & 0x800:
            velocity = velocity_raw - 0x1000
        else:
            velocity = velocity_raw
        raw_uint64 >>= 12
        
        current_raw = raw_uint64 & 0xFFF
        if current_raw & 0x800:
            current = current_raw - 0x1000
        else:
            current = current_raw
        raw_uint64 >>= 12
        
        force_sensor = raw_uint64 & 0xFFF
        
        return {
            'status': status,
            'position': position,
            'velocity': velocity,
            'current': current,
            'force_sensor': force_sensor
        }
    
    def _get_status_description(self, status_code: int) -> str:
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
        if msg is None or len(msg.data) < 8:
            return {'error': f"电机{motor_id}未收到应答或数据长度不足"}
        
        data = msg.data
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
                parsed_fields = self._parse_motor_info_bitfields(data)
                
                position = parsed_fields['position']
                velocity_raw = parsed_fields['velocity']
                current_raw = parsed_fields['current']
                force_sensor = parsed_fields['force_sensor']
                status = parsed_fields['status']
                
                velocity_physical = velocity_raw * 0.001
                current_physical = current_raw * 0.001
                current_ma = current_raw
                
                decoded.update({
                    'status_code': status,
                    'position_raw': position,
                    'velocity_raw': velocity_raw,
                    'current_raw': current_raw,
                    'force_sensor_raw': force_sensor,
                    'current_position': position,
                    'current_speed': velocity_raw,
                    'current_current': current_ma,
                    'status': status,
                    'position_normalized': position / 4095.0,
                    'velocity_physical': velocity_physical,
                    'current_physical': current_physical,
                    'current_ma': current_ma,
                    'force_sensor_adc': force_sensor,
                    'force_sensor_normalized': force_sensor / 4095.0,
                    'status_description': self._get_status_description(status),
                    'is_normal': status == 0,
                    'has_error': status != 0,
                    'units': {
                        'position': '0-4095 (满行程)',
                        'velocity': '0.001行程/s',
                        'current': 'mA',
                        'force_sensor': 'ADC原始值 (0-4095)'
                    }
                })
                
                decoded['c_struct_demo'] = {
                    'description': '对应C结构体MFingerInfo_t的字段',
                    'P': position,
                    'V': velocity_raw,
                    'I': current_raw,
                    'F': force_sensor,
                    'status': status
                }
                
            else:
                decoded['error'] = f"意外的指令类型: 0x{data[0]:02X}"
                
        except Exception as e:
            decoded['error'] = f"解码过程中发生错误: {e}"
            decoded['exception_details'] = str(e)
        
        return decoded
    
    def _decode_tactile_response(self, msg: can.Message, motor_id: int) -> Dict:
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
                tactile_values = list(data[2:8])
                
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
    
    def batch_get_motors_info(self, motor_ids: List[int] = None, timeout: float = 1.0) -> Dict[int, Dict]:
        if motor_ids is None:
            motor_ids = self.motor_ids
        
        if not self._validate_motor_ids(motor_ids):
            return {}
        
        print(f"批量获取电机信息: {motor_ids}")
        
        success_count = 0
        for motor_id in motor_ids:
            if self._send_command(self.COMMAND_READ_MOTOR_INFO, motor_id):
                success_count += 1
        
        if success_count == 0:
            print("所有指令发送失败")
            return {}
        
        responses = self._collect_responses(motor_ids, timeout)
        
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._decode_motor_info_response(msg, motor_id)
        
        return results
    
    def batch_move_motors(self, positions: List[int], speeds: List[int], 
                         current_limits: List[int], motor_ids: List[int] = None, 
                         timeout: float = 1.0) -> Dict[int, Dict]:
        if motor_ids is None:
            motor_ids = self.motor_ids
        
        if not self._validate_motor_ids(motor_ids):
            return {}
        
        if not (len(positions) == len(speeds) == len(current_limits) == len(motor_ids)):
            print("参数列表长度不匹配")
            return {}
        
        print(f"批量移动电机: {motor_ids}")
        
        success_count = 0
        for i, motor_id in enumerate(motor_ids):
            payload = [
                positions[i] & 0xFF,
                (positions[i] >> 8) & 0xFF,
                speeds[i] & 0xFF,
                (speeds[i] >> 8) & 0xFF,
                current_limits[i] & 0xFF,
                (current_limits[i] >> 8) & 0xFF,
                0x00
            ]
            
            if self._send_command(self.COMMAND_CTRL_MOTOR_POSITION, motor_id, payload):
                success_count += 1
                print(f"  电机{motor_id}: 位置={positions[i]}, 速度={speeds[i]}, 电流={current_limits[i]}")
        
        if success_count == 0:
            print("所有位置指令发送失败")
            return {}
        
        responses = self._collect_responses(motor_ids, timeout)
        
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._decode_motor_info_response(msg, motor_id)
        
        return results
    
    def batch_get_tactile_data(self, motor_ids: List[int] = None, 
                              frame_numbers: List[int] = None, 
                              timeout: float = 1.0) -> Dict[int, Dict]:
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
        
        success_count = 0
        for i, motor_id in enumerate(motor_ids):
            payload = [frame_numbers[i]]
            if self._send_command(self.COMMAND_READ_TACTILE_DATA, motor_id, payload):
                success_count += 1
                print(f"  电机{motor_id}: 帧号={frame_numbers[i]}")
        
        if success_count == 0:
            print("所有触觉数据指令发送失败")
            return {}
        
        responses = self._collect_responses(motor_ids, timeout)
        
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._decode_tactile_response(msg, motor_id)
        
        return results
    
    def get_all_positions(self) -> Dict[int, int]:
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
        positions = [position] * len(self.motor_ids)
        speeds = [speed] * len(self.motor_ids)
        current_limits = [current_limit] * len(self.motor_ids)
        return self.batch_move_motors(positions, speeds, current_limits)
    
    def get_all_tactile_data(self, frame_number: int = 0) -> Dict[int, Dict]:
        frame_numbers = [frame_number] * len(self.motor_ids)
        return self.batch_get_tactile_data(frame_numbers=frame_numbers)
    
    def __enter__(self):
        if not self.connected:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
    
    def __del__(self):
        self.disconnect()


