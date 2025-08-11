#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header, String, Float64MultiArray, MultiArrayDimension
import json
import time
import logging
from typing import Dict, List
from rh2_manager import RH2ControllerManager
from can_controller import CANController


class RH2ROSWrapperV2(Node):
    """
    RH2控制器ROS2包装器 V2
    
    统一的三个话题接口:
    - /rh2/{name}/control (订阅String) - 接收JSON格式的控制指令
    - /rh2/{name}/response (发布String) - 发布JSON格式的响应信息  
    - /rh2/{name}/tactile (发布Float64MultiArray) - 发布所有电机的触觉数据
    """
    
    def __init__(self, 
                 name: str,
                 interface: str = 'pcan', 
                 channel: str = 'PCAN_USBBUS1',
                 bitrate: int = 1000000,
                 motor_ids: List[int] = [1, 2, 3, 4, 5, 6],
                 node_name: str = None):
        
        # 设置节点名称
        if node_name is None:
            node_name = f'rh2_{name}_wrapper_v2_node'
        super().__init__(node_name)
        
        # 保存配置参数
        self.name = name
        self.motor_ids = motor_ids
        
        # 配置日志
        self.logger = self.get_logger()
        
        # 初始化CAN控制器和RH2管理器
        try:
            self.can_controller = CANController(
                interface=interface,
                channel=channel, 
                bitrate=bitrate,
                auto_connect=True
            )
            
            self.rh2_manager = RH2ControllerManager(
                can_controller=self.can_controller,
                motor_ids=motor_ids
            )
            
            self.logger.info(f"RH2管理器初始化成功: name={name}, motor_ids={motor_ids}")
        except Exception as e:
            self.logger.error(f"RH2管理器初始化失败: {e}")
            self.rh2_manager = None
            self.can_controller = None
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建三个统一的话题
        # 1. 控制指令订阅者
        control_topic = f'/rh2/{name}/control'
        self.control_subscriber = self.create_subscription(
            String,
            control_topic,
            self.control_callback,
            qos_profile
        )
        
        # 2. 响应信息发布者
        response_topic = f'/rh2/{name}/response'
        self.response_publisher = self.create_publisher(
            String,
            response_topic,
            qos_profile
        )
        
        # 3. 触觉数据发布者
        tactile_topic = f'/rh2/{name}/tactile'
        self.tactile_publisher = self.create_publisher(
            Float64MultiArray,
            tactile_topic,
            qos_profile
        )
        
        # 状态变量
        self.last_motor_info = {}
        self.is_connected = False
        self.command_sequence = 0  # 命令序列号
        
        # 定时器 - 定期发布状态和触觉数据
        self.info_timer = self.create_timer(1.0, self.publish_motor_status)  # 1Hz状态更新
        self.tactile_timer = self.create_timer(0.1, self.publish_tactile_data)  # 10Hz触觉数据
        
        # 检查连接状态
        self.check_connection()
        
        self.logger.info(f"RH2 ROS包装器V2启动完成: {name}")
        self.logger.info(f"话题: {control_topic}, {response_topic}, {tactile_topic}")
    
    def check_connection(self):
        """检查控制器连接状态"""
        if self.rh2_manager:
            self.is_connected = self.rh2_manager.is_connected()
            if self.is_connected:
                self.logger.info("RH2管理器连接正常")
            else:
                self.logger.warning("RH2管理器连接异常，尝试重连...")
                try:
                    self.can_controller.connect()
                    self.is_connected = self.rh2_manager.is_connected()
                except Exception as e:
                    self.logger.error(f"重连失败: {e}")
        else:
            self.is_connected = False
    
    def control_callback(self, msg: String):
        """
        处理统一控制指令回调
        
        消息格式 (JSON):
        {
            "command_id": "unique_id",
            "timestamp": 1234567890.123,
            "command_type": "get_motor_info" | "move_motor" | "get_all_motor_info" | "move_all_motors",
            "motor_id": 1,  // 单个电机操作时使用
            "motor_ids": [1,2,3],  // 多个电机操作时使用
            "parameters": {
                "position": 1000,
                "speed": 500,
                "current_limit": 300,
                "positions": [1000, 2000, 3000],  // 多电机时使用
                "speeds": [500, 600, 700],
                "current_limits": [300, 400, 500]
            }
        }
        """
        try:
            # 解析JSON命令
            command_data = json.loads(msg.data)
            self.logger.info(f"收到控制指令: {command_data}")
            
            if not self.is_connected:
                self.send_error_response(
                    command_data.get('command_id', 'unknown'),
                    "控制器未连接"
                )
                return
            
            command_type = command_data.get('command_type')
            command_id = command_data.get('command_id', f'auto_{int(time.time()*1000)}')
            
            if command_type == "get_motor_info":
                self.handle_get_single_motor_info(command_data, command_id)
                
            elif command_type == "move_motor":
                self.handle_move_single_motor(command_data, command_id)
                
            elif command_type == "get_all_motor_info":
                self.handle_get_all_motor_info(command_data, command_id)
                
            elif command_type == "move_all_motors":
                self.handle_move_all_motors(command_data, command_id)
                
            else:
                self.send_error_response(command_id, f"未知命令类型: {command_type}")
                
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON解析失败: {e}")
            self.send_error_response("unknown", f"JSON解析失败: {e}")
        except Exception as e:
            self.logger.error(f"处理控制指令异常: {e}")
            self.send_error_response("unknown", f"处理指令异常: {e}")
    
    def handle_get_single_motor_info(self, command_data: Dict, command_id: str):
        """处理获取单个电机信息"""
        try:
            motor_id = command_data.get('motor_id')
            if motor_id is None:
                self.send_error_response(command_id, "缺少motor_id参数")
                return
            
            controller = self.rh2_manager.get_controller(motor_id)
            if not controller:
                self.send_error_response(command_id, f"电机{motor_id}控制器不存在")
                return
            
            motor_info = controller.get_motor_info()
            
            response = {
                "command_id": command_id,
                "timestamp": time.time(),
                "response_type": "motor_info",
                "success": True,
                "data": {
                    "motor_id": motor_id,
                    "motor_info": motor_info
                }
            }
            
            self.send_response(response)
            
        except Exception as e:
            self.send_error_response(command_id, f"获取电机{motor_id}信息失败: {e}")
    
    def handle_move_single_motor(self, command_data: Dict, command_id: str):
        """处理移动单个电机"""
        try:
            motor_id = command_data.get('motor_id')
            parameters = command_data.get('parameters', {})
            
            if motor_id is None:
                self.send_error_response(command_id, "缺少motor_id参数")
                return
            
            position = parameters.get('position')
            speed = parameters.get('speed')
            current_limit = parameters.get('current_limit')
            
            if position is None or speed is None or current_limit is None:
                self.send_error_response(command_id, "缺少position, speed或current_limit参数")
                return
            
            controller = self.rh2_manager.get_controller(motor_id)
            if not controller:
                self.send_error_response(command_id, f"电机{motor_id}控制器不存在")
                return
            
            move_result = controller.move_motor(int(position), int(speed), int(current_limit))
            
            response = {
                "command_id": command_id,
                "timestamp": time.time(),
                "response_type": "move_motor",
                "success": True,
                "data": {
                    "motor_id": motor_id,
                    "move_result": move_result,
                    "parameters": {
                        "position": position,
                        "speed": speed,
                        "current_limit": current_limit
                    }
                }
            }
            
            self.send_response(response)
            
        except Exception as e:
            self.send_error_response(command_id, f"移动电机{motor_id}失败: {e}")
    
    def handle_get_all_motor_info(self, command_data: Dict, command_id: str):
        """处理获取所有电机信息"""
        try:
            motor_info = self.rh2_manager.get_all_motor_info()
            
            response = {
                "command_id": command_id,
                "timestamp": time.time(),
                "response_type": "all_motor_info",
                "success": True,
                "data": {
                    "motor_info": motor_info
                }
            }
            
            self.send_response(response)
            
        except Exception as e:
            self.send_error_response(command_id, f"获取所有电机信息失败: {e}")
    
    def handle_move_all_motors(self, command_data: Dict, command_id: str):
        """处理移动所有电机"""
        try:
            parameters = command_data.get('parameters', {})
            
            positions = parameters.get('positions', [])
            speeds = parameters.get('speeds', [])
            current_limits = parameters.get('current_limits', [])
            
            if not (positions and speeds and current_limits):
                self.send_error_response(command_id, "缺少positions, speeds或current_limits参数")
                return
            
            if not (len(positions) == len(speeds) == len(current_limits)):
                self.send_error_response(command_id, "positions, speeds, current_limits长度不一致")
                return
            
            move_results = self.rh2_manager.move_all_motors(
                [int(p) for p in positions],
                [int(s) for s in speeds], 
                [int(c) for c in current_limits]
            )
            
            response = {
                "command_id": command_id,
                "timestamp": time.time(),
                "response_type": "move_all_motors",
                "success": True,
                "data": {
                    "move_results": move_results,
                    "parameters": {
                        "positions": positions,
                        "speeds": speeds,
                        "current_limits": current_limits
                    }
                }
            }
            
            self.send_response(response)
            
        except Exception as e:
            self.send_error_response(command_id, f"移动所有电机失败: {e}")
    
    def send_response(self, response_data: Dict):
        """发送响应消息"""
        try:
            response_msg = String()
            response_msg.data = json.dumps(response_data, ensure_ascii=False)
            self.response_publisher.publish(response_msg)
            self.logger.info(f"发送响应: {response_data['command_id']}")
        except Exception as e:
            self.logger.error(f"发送响应失败: {e}")
    
    def send_error_response(self, command_id: str, error_message: str):
        """发送错误响应"""
        response = {
            "command_id": command_id,
            "timestamp": time.time(),
            "response_type": "error",
            "success": False,
            "error": error_message
        }
        self.send_response(response)
    
    def publish_motor_status(self):
        """定期发布电机状态"""
        if not self.is_connected:
            self.check_connection()
            return
        
        try:
            motor_info = self.rh2_manager.get_all_motor_info()
            if motor_info:
                self.last_motor_info = motor_info
                
                # 发送状态更新响应
                status_response = {
                    "command_id": f"status_update_{int(time.time()*1000)}",
                    "timestamp": time.time(),
                    "response_type": "status_update",
                    "success": True,
                    "data": {
                        "motor_info": motor_info,
                        "connection_status": self.is_connected
                    }
                }
                
                self.send_response(status_response)
                
        except Exception as e:
            self.logger.debug(f"电机状态更新失败: {e}")
    
    def publish_tactile_data(self):
        """定期发布触觉数据"""
        if not self.is_connected:
            return
        
        try:
            tactile_data = self.rh2_manager.get_tactile_data()
            self.logger.info(f"电机{list(tactile_data.keys())}触觉数据: {tactile_data}")
            
            # 将所有电机的触觉数据组合成一个大数组
            # 格式: [motor1_sensor1, motor1_sensor2, ..., motor2_sensor1, motor2_sensor2, ...]
            all_tactile_values = []
            motor_info = []
            
            for motor_id in sorted(tactile_data.keys()):
                sensor_values = tactile_data[motor_id]
                all_tactile_values.extend(sensor_values)
                motor_info.append({
                    'motor_id': motor_id,
                    'sensor_count': len(sensor_values),
                    'start_index': len(all_tactile_values) - len(sensor_values)
                })
            
            # 创建Float64MultiArray消息
            msg = Float64MultiArray()
            msg.data = [float(val) for val in all_tactile_values]
            
            # 设置数组维度信息
            # 第一个维度：电机数量
            motor_dim = MultiArrayDimension()
            motor_dim.label = "motors"
            motor_dim.size = len(tactile_data)
            motor_dim.stride = len(all_tactile_values)
            
            # 第二个维度：每个电机的传感器数量（假设都是8个）
            sensor_dim = MultiArrayDimension()
            sensor_dim.label = "sensors_per_motor"
            sensor_dim.size = 8  # 每个电机8个传感器
            sensor_dim.stride = 8
            
            msg.layout.dim = [motor_dim, sensor_dim]
            msg.layout.data_offset = 0
            
            # 发布触觉数据
            self.tactile_publisher.publish(msg)
            
            # 记录详细信息到日志
            if len(all_tactile_values) > 0:
                self.logger.info(f"发布触觉数据: {len(tactile_data)}个电机, 总共{len(all_tactile_values)}个传感器值")
                for info in motor_info:
                    start_idx = info['start_index']
                    end_idx = start_idx + info['sensor_count']
                    values = all_tactile_values[start_idx:end_idx]
                    self.logger.info(f"  电机{info['motor_id']}: {values}")
                    
        except Exception as e:
            self.logger.debug(f"触觉数据发布失败: {e}")
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def shutdown(self):
        """关闭包装器"""
        self.logger.info("关闭RH2 ROS包装器V2")
        if self.rh2_manager:
            self.rh2_manager.disconnect()
        self.destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # 创建RH2 ROS包装器V2节点
        wrapper_name = "rh2_hand"
        motor_ids = [1, 2, 3, 4, 5, 6]
        
        rh2_wrapper = RH2ROSWrapperV2(
            name=wrapper_name,
            interface='pcan',
            channel='PCAN_USBBUS1', 
            bitrate=1000000,
            motor_ids=motor_ids
        )
        
        print(f"RH2 ROS包装器V2已启动: {wrapper_name}")
        print("统一话题接口:")
        print(f"  订阅: /rh2/{wrapper_name}/control (String) - JSON格式控制指令")
        print(f"  发布: /rh2/{wrapper_name}/response (String) - JSON格式响应信息")
        print(f"  发布: /rh2/{wrapper_name}/tactile (Float64MultiArray) - 所有电机触觉数据")
        print("按 Ctrl+C 退出")
        
        # 运行节点
        rclpy.spin(rh2_wrapper)
        
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭...")
    except Exception as e:
        print(f"运行时错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'rh2_wrapper' in locals():
            rh2_wrapper.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
