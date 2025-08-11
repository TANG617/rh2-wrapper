#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import json
import time
import logging
from typing import Dict, List
from rh2_manager import RH2ControllerManager
from can_controller import CANController

# 自定义消息类型 (简化版，使用标准消息)
class RH2ControlCommand:
    """RH2控制指令消息"""
    def __init__(self):
        self.header = Header()
        self.motor_ids = []      # List[int]
        self.positions = []      # List[int] 
        self.speeds = []         # List[int]
        self.current_limits = [] # List[int]
        self.command_type = ""   # str: "move_motors" or "get_motors_info"

class RH2Response:
    """RH2响应消息"""
    def __init__(self):
        self.header = Header()
        self.motor_data = {}     # Dict[int, Dict] - 电机数据
        self.success = False     # bool
        self.error_message = ""  # str

class RH2ROSWrapper(Node):
    """
    RH2控制器ROS2包装器
    
    功能:
    - 封装RH2ControllerManager的所有功能
    - 提供ROS2 topic接口
    - 支持电机信息查询、运动控制和触觉数据发布
    
    Topics:
    - /rh2/{name}/control/motor_{motor_id} (订阅) - 接收控制指令
    - /rh2/{name}/info/motor_{motor_id} (发布) - 发布电机信息
    - /rh2/{name}/tactile/motor_{motor_id} (发布) - 发布触觉数据
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
            node_name = f'rh2_{name}_wrapper_node'
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
        
        # 创建每个电机的发布者和订阅者
        self.control_subscribers = {}
        self.info_publishers = {}
        self.tactile_publishers = {}
        
        for motor_id in motor_ids:
            # 控制指令订阅者
            control_topic = f'/rh2/{name}/control/motor_{motor_id}'
            self.control_subscribers[motor_id] = self.create_subscription(
                JointState,
                control_topic,
                lambda msg, mid=motor_id: self.motor_control_callback(msg, mid),
                qos_profile
            )
            
            # 电机信息发布者
            info_topic = f'/rh2/{name}/info/motor_{motor_id}'
            self.info_publishers[motor_id] = self.create_publisher(
                JointState,
                info_topic,
                qos_profile
            )
            
            # 触觉数据发布者
            tactile_topic = f'/rh2/{name}/tactile/motor_{motor_id}'
            self.tactile_publishers[motor_id] = self.create_publisher(
                Float64MultiArray,
                tactile_topic,
                qos_profile
            )
        
        # 状态变量
        self.last_motor_info = {}
        self.is_connected = False
        
        # 定时器 - 定期发布状态和触觉数据
        self.info_timer = self.create_timer(0.1, self.publish_motor_info)  # 10Hz
        self.tactile_timer = self.create_timer(0.05, self.publish_tactile_data)  # 20Hz
        
        # 检查连接状态
        self.check_connection()
        
        self.logger.info(f"RH2 ROS包装器启动完成: {name}")
    
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
    
    def motor_control_callback(self, msg: JointState, motor_id: int):
        """
        处理单个电机控制指令回调
        
        JointState消息格式:
        - name[0]: 命令类型 ("move_motor" 或 "get_motor_info")
        - position[0]: 目标位置 (move_motor时使用)
        - velocity[0]: 速度 (move_motor时使用) 
        - effort[0]: 电流限制 (move_motor时使用)
        """
        try:
            if not self.is_connected:
                self.logger.error(f"控制器未连接，忽略电机{motor_id}指令")
                self.publish_motor_error_response(motor_id, "控制器未连接")
                return
            
            if len(msg.name) == 0:
                self.logger.error(f"电机{motor_id}无效的指令格式：缺少命令类型")
                self.publish_motor_error_response(motor_id, "无效的指令格式")
                return
            
            command_type = msg.name[0]
            self.logger.debug(f"电机{motor_id}收到指令: {command_type}")
            
            if command_type == "get_motor_info":
                self.handle_get_motor_info(motor_id)
                
            elif command_type == "move_motor":
                self.handle_move_motor(motor_id, msg)
                
            else:
                self.logger.warning(f"电机{motor_id}未知指令类型: {command_type}")
                self.publish_motor_error_response(motor_id, f"未知指令类型: {command_type}")
                
        except Exception as e:
            self.logger.error(f"处理电机{motor_id}指令时发生异常: {e}")
            self.publish_motor_error_response(motor_id, f"处理指令异常: {e}")
    
    def handle_get_motor_info(self, motor_id: int):
        """处理获取单个电机信息指令"""
        try:
            self.logger.debug(f"执行获取电机{motor_id}信息")
            controller = self.rh2_manager.get_controller(motor_id)
            
            if controller:
                motor_info = controller.get_motor_info()
                if motor_info and 'error' not in motor_info:
                    self.publish_single_motor_info_response(motor_id, motor_info, "get_motor_info")
                    self.logger.debug(f"电机{motor_id}信息获取成功")
                else:
                    self.logger.warning(f"电机{motor_id}信息获取失败")
                    self.publish_motor_error_response(motor_id, f"电机{motor_id}信息获取失败")
            else:
                self.publish_motor_error_response(motor_id, f"电机{motor_id}控制器不存在")
                
        except Exception as e:
            self.logger.error(f"获取电机{motor_id}信息异常: {e}")
            self.publish_motor_error_response(motor_id, f"获取电机信息异常: {e}")
    
    def handle_move_motor(self, motor_id: int, msg: JointState):
        """处理移动单个电机指令"""
        try:
            # 验证数据长度
            if not (len(msg.position) >= 1 and len(msg.velocity) >= 1 and len(msg.effort) >= 1):
                error_msg = f"参数长度不足: 需要位置、速度、电流参数"
                self.logger.error(error_msg)
                self.publish_motor_error_response(motor_id, error_msg)
                return
            
            # 转换为整数
            position = int(msg.position[0])
            speed = int(msg.velocity[0]) 
            current_limit = int(msg.effort[0])
            
            self.logger.info(f"执行电机{motor_id}移动: 位置={position}, 速度={speed}, 电流={current_limit}")
            
            # 获取控制器并执行移动指令
            controller = self.rh2_manager.get_controller(motor_id)
            if controller:
                move_result = controller.move_motor(position, speed, current_limit)
                
                if move_result and 'error' not in move_result:
                    self.publish_single_motor_info_response(motor_id, move_result, "move_motor")
                    self.logger.debug(f"电机{motor_id}移动指令执行成功")
                else:
                    self.logger.warning(f"电机{motor_id}移动指令执行失败")
                    self.publish_motor_error_response(motor_id, f"电机{motor_id}移动指令执行失败")
            else:
                self.publish_motor_error_response(motor_id, f"电机{motor_id}控制器不存在")
                
        except Exception as e:
            self.logger.error(f"移动电机{motor_id}异常: {e}")
            self.publish_motor_error_response(motor_id, f"移动电机异常: {e}")
    
    def publish_single_motor_info_response(self, motor_id: int, motor_info: Dict, command_type: str):
        """发布单个电机信息响应"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"rh2_motor_{motor_id}"
            
            # 设置关节名称
            msg.name = [command_type, f"motor_{motor_id}"]
            
            # 提取位置、速度、电流数据
            if 'error' not in motor_info:
                msg.position = [float(motor_info.get('current_position', 0))]
                msg.velocity = [float(motor_info.get('current_speed', 0))]
                msg.effort = [float(motor_info.get('current_current', 0))]
            else:
                # 错误情况下填充NaN
                msg.position = [float('nan')]
                msg.velocity = [float('nan')] 
                msg.effort = [float('nan')]
            
            # 发布到对应的topic
            if motor_id in self.info_publishers:
                self.info_publishers[motor_id].publish(msg)
                self.logger.debug(f"发布电机{motor_id}响应: {command_type}")
            
        except Exception as e:
            self.logger.error(f"发布电机{motor_id}响应失败: {e}")
    
    def publish_motor_error_response(self, motor_id: int, error_message: str):
        """发布单个电机错误响应"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"rh2_motor_{motor_id}"
            
            msg.name = ["error", error_message]
            msg.position = []
            msg.velocity = []
            msg.effort = []
            
            # 发布到对应的info topic
            if motor_id in self.info_publishers:
                self.info_publishers[motor_id].publish(msg)
                self.logger.debug(f"发布电机{motor_id}错误响应: {error_message}")
            
        except Exception as e:
            self.logger.error(f"发布电机{motor_id}错误响应失败: {e}")
    
    def publish_motor_info(self):
        """定期发布电机信息"""
        if not self.is_connected:
            self.check_connection()
            return
        
        # 定期获取电机状态 (降低频率避免过载)
        current_time = time.time()
        if not hasattr(self, '_last_info_time') or current_time - self._last_info_time > 1.0:
            self._last_info_time = current_time
            try:
                motor_info = self.rh2_manager.get_all_motor_info()
                if motor_info:
                    self.last_motor_info = motor_info
                    # 为每个电机发布信息
                    for motor_id, info in motor_info.items():
                        self.publish_single_motor_info_response(motor_id, info, "status_update")
            except Exception as e:
                self.logger.debug(f"电机信息更新失败: {e}")
    
    def publish_tactile_data(self):
        """定期发布触觉数据"""
        if not self.is_connected:
            return
        
        try:
            tactile_data = self.rh2_manager.get_tactile_data()
            
            for motor_id, sensor_values in tactile_data.items():
                if motor_id in self.tactile_publishers:
                    self.logger.info(f"电机{motor_id}触觉数据: {len(sensor_values)}个传感器, 值: {sensor_values}")
                    
                    # 创建Float64MultiArray消息
                    msg = Float64MultiArray()
                    msg.data = [float(val) for val in sensor_values]
                    
                    # 设置数组维度信息
                    dim = MultiArrayDimension()
                    dim.label = f"tactile_sensors_motor_{motor_id}"
                    dim.size = len(sensor_values)
                    dim.stride = len(sensor_values)
                    msg.layout.dim = [dim]
                    msg.layout.data_offset = 0
                    
                    # 发布触觉数据
                    self.tactile_publishers[motor_id].publish(msg)
                    
        except Exception as e:
            self.logger.debug(f"触觉数据发布失败: {e}")
    
    def get_motor_status_dict(self) -> Dict:
        """获取当前电机状态字典格式"""
        return self.last_motor_info
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def shutdown(self):
        """关闭包装器"""
        self.logger.info("关闭RH2 ROS包装器")
        if self.rh2_manager:
            self.rh2_manager.disconnect()
        self.destroy_node()


def create_motor_command_message(command_type: str, position: int = None, 
                                speed: int = None, current_limit: int = None) -> JointState:
    """
    创建单个电机控制指令消息的辅助函数
    
    Args:
        command_type: "get_motor_info" 或 "move_motor"
        position: 目标位置 (move_motor时需要)
        speed: 速度 (move_motor时需要)
        current_limit: 电流限制 (move_motor时需要)
    
    Returns:
        JointState消息
    """
    msg = JointState()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.header.frame_id = "rh2_command"
    
    msg.name = [command_type]
    
    if command_type == "move_motor" and position is not None and speed is not None and current_limit is not None:
        msg.position = [float(position)]
        msg.velocity = [float(speed)]
        msg.effort = [float(current_limit)]
    else:
        msg.position = []
        msg.velocity = []
        msg.effort = []
    
    return msg


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # 创建RH2 ROS包装器节点
        wrapper_name = "rh2_hand"
        motor_ids = [1, 2, 3, 4, 5, 6]
        
        rh2_wrapper = RH2ROSWrapper(
            name=wrapper_name,
            interface='pcan',
            channel='PCAN_USBBUS1', 
            bitrate=1000000,
            motor_ids=motor_ids
        )
        
        print(f"RH2 ROS包装器已启动: {wrapper_name}")
        print("Topics:")
        for motor_id in motor_ids:
            print(f"  订阅: /rh2/{wrapper_name}/control/motor_{motor_id} (JointState)")
            print(f"  发布: /rh2/{wrapper_name}/info/motor_{motor_id} (JointState)")
            print(f"  发布: /rh2/{wrapper_name}/tactile/motor_{motor_id} (Float64MultiArray)")
        print("按 Ctrl+C 退出")
        
        # 运行节点
        rclpy.spin(rh2_wrapper)
        
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭...")
    except Exception as e:
        print(f"运行时错误: {e}")
    finally:
        if 'rh2_wrapper' in locals():
            rh2_wrapper.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
