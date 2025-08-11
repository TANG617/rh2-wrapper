#!/usr/bin/env python3
# Tactile sensor is not fully implemented
import can
import time
import struct
import logging
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class RH2Controller:
    
    COMMAND_READ_MOTOR_INFO = 0xA0
    COMMAND_CTRL_MOTOR_POSITION = 0xAA
    COMMAND_READ_TACTILE_DATA = 0xB6
    
    def __init__(self, interface: str = 'pcan', channel: str = 'PCAN_USBBUS1', 
                 bitrate: int = 1000000, auto_connect: bool = True, 
                 motor_ids: List[int] = [1, 2, 3, 4, 5, 6]):
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
            logger.info(f"CAN总线连接成功: {self.interface}:{self.channel}")
            return True
        except Exception as e:
            logger.error(f"CAN总线连接失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        if self.bus and self.connected:
            self.bus.shutdown()
            self.connected = False
            logger.info("CAN总线已断开")
    
    def is_connected(self) -> bool:
        return self.connected
    
    def _validate_motor_ids(self, motor_ids: List[int]) -> bool:
        for motor_id in motor_ids:
            if motor_id not in self.motor_ids:
                logger.warning(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
                return False
        return True
    
    def _send_command(self, command: int, motor_id: int, data_payload: List[int] = None) -> bool:
        if not self.connected:
            logger.error("CAN总线未连接")
            return False

        if motor_id not in self.motor_ids:
            logger.warning(f"无效的电机ID: {motor_id}, 有效范围: {self.motor_ids}")
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
            logger.error(f"发送失败 (电机{motor_id}): {e}")
            return False
    
    def _collect_responses(self,  timeout: float = 1.0) -> Dict[int, Optional[can.Message]]:
        results = {}
        expected_response_ids = {0x100 + motor_id: motor_id for motor_id in self.motor_ids}
        
        start_time = time.time()
        while time.time() - start_time < timeout and len(results) < len(self.motor_ids):
            rx = self.bus.recv(timeout=0.1)
            if rx is None:
                continue
            
            motor_id = expected_response_ids.get(rx.arbitration_id)
            if motor_id is not None and motor_id not in results:
                results[motor_id] = rx
        
        for motor_id in self.motor_ids:
            if motor_id not in results:
                results[motor_id] = None
        
        return results
    
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

    def _parse_motor_info(self, msg: can.Message, motor_id: int) -> Dict:
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
    
    def _parse_tactile_info(self, msg: can.Message, motor_id: int) -> Dict:
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
    
    def get_motors_info(self, timeout: float = 1.0) -> Dict[int, Dict]:
        logger.debug(f"获取所有电机信息: {self.motor_ids}")
        
        success_count = 0
        for motor_id in self.motor_ids:
            if self._send_command(self.COMMAND_READ_MOTOR_INFO, motor_id):
                success_count += 1
        
        if success_count == 0:
            logger.error("获取所有电机信息失败")
            return {}
        
        responses = self._collect_responses(timeout)
        
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._parse_motor_info(msg, motor_id)
        
        for motor_id, info in results.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                speed = info.get('current_speed', 'N/A')
                current = info.get('current_current', 'N/A')
                logger.info(f"电机{motor_id}: 当前位置={pos}, 当前速度={speed}, 当前电流={current}mA")
            else:
                logger.error(f"电机{motor_id}: {info['error']}")
        
        return results

    def get_tactile_info(self, frame_numbers: List[int] = [0,1,2,3,4,5], 
                              timeout: float = 1.0) -> Dict[int, Dict]:
        logger.debug(f"获取全部触觉数据: {self.motor_ids}")
        success_count = 0
        for i, motor_id in enumerate(self.motor_ids):
            payload = [frame_numbers[i]]
            if self._send_command(self.COMMAND_READ_TACTILE_DATA, motor_id, payload):
                success_count += 1
                logger.debug(f"电机{motor_id}: 帧号={frame_numbers[i]}")
        
        if success_count == 0:
            logger.error("所有触觉数据指令发送失败")
            return {}
        
        responses = self._collect_responses(timeout)
        
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._parse_tactile_info(msg, motor_id)
        

        for tactile_frame_id, info in results.items():
            if 'error' not in info:
                tactile_values = info.get('tactile_values', 'N/A')
                logger.info(f"电机{tactile_frame_id}: 触觉数据={tactile_values}")
            else:
                logger.error(f"电机{motor_id}: {info['error']}")
        
        return results

    def get_tactile_data_frames(self, motor_id: int, timeout: float = 2.0) -> Dict:
        """
        使用0xb6指令读取单个电机的多帧触觉数据
        
        指令格式：
        - byte 0: 0xb6 
        - byte 1[6:0]: 帧号，从0开始递增
        
        回复格式：
        - byte 0: 0xb6
        - byte 1[7]: 1代表最后一帧
        - byte 1[6:0]: 帧号
        - byte 2-7: 触觉数据
        
        Args:
            motor_id: 电机ID
            timeout: 超时时间（秒）
            
        Returns:
            包含所有帧数据的字典
        """
        if not self.connected:
            logger.error("CAN总线未连接")
            return {'error': 'CAN总线未连接'}
            
        if motor_id not in self.motor_ids:
            logger.error(f"无效的电机ID: {motor_id}")
            return {'error': f'无效的电机ID: {motor_id}'}
        
        logger.info(f"开始读取电机{motor_id}的多帧触觉数据")
        
        all_frames = {}
        frame_number = 0
        expected_response_id = 0x100 + motor_id
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 发送当前帧号的读取指令
            payload = [frame_number & 0x7F]  # byte1[6:0]是帧号
            
            if not self._send_command(self.COMMAND_READ_TACTILE_DATA, motor_id, payload):
                logger.error(f"发送帧{frame_number}读取指令失败")
                break
                
            logger.debug(f"发送读取帧{frame_number}指令到电机{motor_id}")
            
            # 等待响应
            frame_start_time = time.time()
            frame_timeout = min(1.0, timeout - (time.time() - start_time))
            
            while time.time() - frame_start_time < frame_timeout:
                rx = self.bus.recv(timeout=0.1)
                if rx is None:
                    continue
                    
                if rx.arbitration_id == expected_response_id:
                    data = rx.data
                    
                    # 验证响应格式 - 对于触觉数据，可能只有6字节而不是8字节
                    if len(data) < 6 or data[0] != self.COMMAND_READ_TACTILE_DATA:
                        logger.warning(f"无效的响应格式: {[hex(x) for x in data]}")
                        continue
                    
                    # 解析帧信息
                    response_frame_number = data[1] & 0x7F  # byte1[6:0]是帧号
                    is_last_frame = bool(data[1] & 0x80)    # byte1[7]是最后一帧标志
                    # 触觉数据从byte2开始，长度根据实际数据长度确定
                    tactile_data = list(data[2:]) if len(data) > 2 else []
                    
                    # 存储帧数据
                    frame_info = {
                        'frame_number': response_frame_number,
                        'is_last_frame': is_last_frame,
                        'tactile_data': tactile_data,
                        'raw_data': [hex(x) for x in data],
                        'raw_bytes': list(data),
                        'timestamp': time.time()
                    }
                    
                    all_frames[response_frame_number] = frame_info
                    
                    logger.info(f"收到帧{response_frame_number}: 触觉数据={tactile_data}, 最后一帧={is_last_frame}")
                    
                    # 如果是最后一帧，结束循环
                    if is_last_frame:
                        logger.info(f"读取完成，共收到{len(all_frames)}帧数据")
                        return {
                            'motor_id': motor_id,
                            'total_frames': len(all_frames),
                            'frames': all_frames,
                            'success': True,
                            'all_raw_data': [frame['raw_data'] for frame in all_frames.values()]
                        }
                    
                    # 准备读取下一帧
                    frame_number += 1
                    break
            else:
                logger.warning(f"等待帧{frame_number}超时")
                break
        
        # 如果到这里说明超时或出错
        if all_frames:
            logger.warning(f"部分成功：收到{len(all_frames)}帧数据，但可能未完整")
            return {
                'motor_id': motor_id,
                'total_frames': len(all_frames),
                'frames': all_frames,
                'success': False,
                'error': '数据读取不完整或超时',
                'all_raw_data': [frame['raw_data'] for frame in all_frames.values()]
            }
        else:
            logger.error(f"读取电机{motor_id}触觉数据完全失败")
            return {
                'motor_id': motor_id,
                'success': False,
                'error': '未收到任何触觉数据'
            }
        
    def move_motors(self, positions: List[int], speeds: List[int], 
                         current_limits: List[int], 
                         timeout: float = 1.0) -> Dict[int, Dict]:
        
        if not (len(positions) == len(speeds) == len(current_limits) <= len(self.motor_ids)):
            logger.warning("参数列表长度小于电机数量")
        
        logger.debug(f"批量移动电机: {self.motor_ids}")
        
        success_count = 0
        for i, motor_id in enumerate(self.motor_ids):
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
        
        if success_count == 0:
            logger.error("所有位置指令发送失败")
            return {}
        
        responses = self._collect_responses(timeout)
        
        results = {}
        for motor_id, msg in responses.items():
            results[motor_id] = self._parse_motor_info(msg, motor_id)
        
        for motor_id, info in results.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                speed = info.get('current_speed', 'N/A')
                current = info.get('current_current', 'N/A')
                logger.info(f"电机{motor_id}: 当前位置={pos}, 当前速度={speed}, 当前电流={current}mA")
            else:
                logger.error(f"电机{motor_id}: {info['error']}")

        return results
    
    def __enter__(self):
        if not self.connected:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
    
    def __del__(self):
        self.disconnect()


