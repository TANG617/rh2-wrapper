#!/usr/bin/env python3
import can
import time
import struct
import logging
from typing import Dict, List, Optional
from can_controller import CANController

logger = logging.getLogger(__name__)

class RH2Controller:
    
    COMMAND_READ_MOTOR_INFO = 0xA0
    COMMAND_CTRL_MOTOR_POSITION = 0xAA
    COMMAND_READ_TACTILE_DATA = 0xB6
    
    def __init__(self, motor_id: int, can_controller: CANController):
        self.motor_id = motor_id
        self.can_controller = can_controller
        self.response_id = 0x100 + motor_id
        self.command_names = {
            0xA0: "读取电机信息",
            0xAA: "位置速度电流混合控制", 
            0xB6: "读触觉数据"
        }
    
    def is_connected(self) -> bool:
        return self.can_controller.is_connected()
    
    def _send_command(self, command: int, data_payload: List[int] = None) -> bool:
        if not self.is_connected():
            return False
        
        data = [command]
        if data_payload:
            data.extend(data_payload[:7])
        
        return self.can_controller.send_message(self.motor_id, data)
    
    def _parse_response_bitfields(self, data: bytes) -> Dict:
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

    def _parse_motor_info(self, msg: can.Message) -> Dict:
        if msg is None or len(msg.data) < 8:
            return {'error': f"电机{self.motor_id}未收到应答或数据长度不足"}
        
        data = msg.data
        decoded = {
            'motor_id': self.motor_id,
            'command_type': hex(data[0]),
            'command_name': self.command_names.get(data[0], f"未知指令({hex(data[0])})"),
            'timestamp': time.time(),
            'raw_data': [hex(x) for x in data],
            'raw_bytes': list(data)
        }
        
        try:
            if data[0] in [self.COMMAND_READ_MOTOR_INFO, self.COMMAND_CTRL_MOTOR_POSITION]:
                parsed_fields = self._parse_response_bitfields(data)
                
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
                
            else:
                decoded['error'] = f"意外的指令类型: 0x{data[0]:02X}"
                
        except Exception as e:
            decoded['error'] = f"解码过程中发生错误: {e}"
            decoded['exception_details'] = str(e)
        
        return decoded
  
    def _decode_tactile_sensors(self, frames_data: Dict) -> Dict:
        all_tactile_bytes = []
        sorted_frames = sorted(frames_data.items())
        
        for frame_num, frame_info in sorted_frames:
            tactile_data = frame_info.get('tactile_data', [])
            all_tactile_bytes.extend(tactile_data)
        
        sensors = {}
        if len(all_tactile_bytes) >= 16:
            for i in range(8):
                byte_index = i * 2
                if byte_index + 1 < len(all_tactile_bytes):
                    low_byte = all_tactile_bytes[byte_index]
                    high_byte = all_tactile_bytes[byte_index + 1]
                    sensor_value = (high_byte << 8) | low_byte
                    sensors[f'sensor_{i}'] = {
                        'value': sensor_value,
                        'raw_bytes': [low_byte, high_byte],
                        'byte_index': byte_index
                    }
        
        return {
            'sensors': sensors,
            'total_bytes': len(all_tactile_bytes),
            'raw_bytes': all_tactile_bytes,
            'sensor_count': len(sensors)
        }
    
    def get_motor_info(self, timeout: float = 1.0) -> Dict:
        if not self._send_command(self.COMMAND_READ_MOTOR_INFO):
            return {'error': '指令发送失败'}
        
        responses = self.can_controller.collect_responses([self.response_id], timeout)
        response = responses.get(self.response_id)
        
        return self._parse_motor_info(response)

    def get_tactile_data_frames(self, timeout: float = 1.0) -> Dict:
        if not self.is_connected():
            return {'error': 'CAN总线未连接'}
        
        all_frames = {}
        frame_number = 0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            payload = [frame_number & 0x7F]
            
            if not self._send_command(self.COMMAND_READ_TACTILE_DATA, payload):
                break
            
            frame_start_time = time.time()
            frame_timeout = min(0.2, timeout - (time.time() - start_time))
            
            while time.time() - frame_start_time < frame_timeout:
                rx = self.can_controller.receive_message(timeout=0.05)
                if rx is None:
                    continue
                    
                if rx.arbitration_id == self.response_id:
                    data = rx.data
                    
                    if len(data) < 6 or data[0] != self.COMMAND_READ_TACTILE_DATA:
                        continue
                    
                    response_frame_number = data[1] & 0x7F
                    is_last_frame = bool(data[1] & 0x80)
                    tactile_data = list(data[2:]) if len(data) > 2 else []
                    
                    frame_info = {
                        'frame_number': response_frame_number,
                        'is_last_frame': is_last_frame,
                        'tactile_data': tactile_data,
                        'raw_data': [hex(x) for x in data],
                        'raw_bytes': list(data),
                        'timestamp': time.time()
                    }
                    
                    all_frames[response_frame_number] = frame_info
                    
                    if is_last_frame:
                        decoded_sensors = self._decode_tactile_sensors(all_frames)
                        
                        return {
                            'motor_id': self.motor_id,
                            'total_frames': len(all_frames),
                            'frames': all_frames,
                            'tactile_sensors': decoded_sensors,
                            'success': True,
                            'all_raw_data': [frame['raw_data'] for frame in all_frames.values()]
                        }
                    
                    frame_number += 1
                    break
            else:
                break
        
        if all_frames:
            decoded_sensors = self._decode_tactile_sensors(all_frames)
            
            return {
                'motor_id': self.motor_id,
                'total_frames': len(all_frames),
                'frames': all_frames,
                'tactile_sensors': decoded_sensors,
                'success': False,
                'error': '数据读取不完整或超时',
                'all_raw_data': [frame['raw_data'] for frame in all_frames.values()]
            }
        else:
            return {
                'motor_id': self.motor_id,
                'success': False,
                'error': '未收到任何触觉数据'
            }

    def get_tactile_data(self, timeout: float = 0.5) -> List[int]:
        try:
            result = self.get_tactile_data_frames(timeout)
            
            if result.get('success') and 'tactile_sensors' in result:
                sensors = result['tactile_sensors'].get('sensors', {})
                finger_sensors = []
                for i in range(8):
                    sensor_key = f'sensor_{i}'
                    if sensor_key in sensors:
                        finger_sensors.append(sensors[sensor_key]['value'])
                    else:
                        finger_sensors.append(0)
                return finger_sensors
            else:
                return [0, 0, 0, 0, 0, 0, 0, 0]
                
        except Exception as e:
            return [0, 0, 0, 0, 0, 0, 0, 0]
 
    def move_motor(self, position: int, speed: int, current_limit: int, timeout: float = 1.0) -> Dict:
        payload = [
            position & 0xFF,
            (position >> 8) & 0xFF,
            speed & 0xFF,
            (speed >> 8) & 0xFF,
            current_limit & 0xFF,
            (current_limit >> 8) & 0xFF,
            0x00
        ]
        
        if not self._send_command(self.COMMAND_CTRL_MOTOR_POSITION, payload):
            return {'error': '指令发送失败'}
        
        responses = self.can_controller.collect_responses([self.response_id], timeout)
        response = responses.get(self.response_id)
        
        return self._parse_motor_info(response)


