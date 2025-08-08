#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import json
import time
import logging
from typing import Dict, List
from rh2_controller import RH2Controller

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
    - 封装RH2Controller的所有非触觉功能
    - 提供ROS2 topic接口
    - 支持电机信息查询和运动控制
    
    Topics:
    - /rh2/controller (订阅) - 接收控制指令
    - /rh2/response (发布) - 发布响应数据
    """
    
    def __init__(self, 
                 interface: str = 'pcan', 
                 channel: str = 'PCAN_USBBUS1',
                 bitrate: int = 1000000,
                 motor_ids: List[int] = [1, 2, 3, 4, 5, 6],
                 node_name: str = 'rh2_controller_node'):
        
        super().__init__(node_name)
        
        # 配置日志
        self.logger = self.get_logger()
        
        # 初始化RH2控制器
        try:
            self.rh2_controller = RH2Controller(
                interface=interface,
                channel=channel, 
                bitrate=bitrate,
                auto_connect=True,
                motor_ids=motor_ids
            )
            self.logger.info(f"RH2控制器初始化成功: motor_ids={motor_ids}")
        except Exception as e:
            self.logger.error(f"RH2控制器初始化失败: {e}")
            self.rh2_controller = None
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建发布者和订阅者
        self.response_publisher = self.create_publisher(
            JointState, 
            '/rh2/response', 
            qos_profile
        )
        
        self.command_subscriber = self.create_subscription(
            JointState,
            '/rh2/controller',
            self.command_callback,
            qos_profile
        )
        
        # 状态变量
        self.last_motor_info = {}
        self.is_connected = False
        
        # 定时器 - 定期发布状态
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10Hz
        
        # 检查连接状态
        self.check_connection()
        
        self.logger.info("RH2 ROS包装器启动完成")
    
    def check_connection(self):
        """检查控制器连接状态"""
        if self.rh2_controller:
            self.is_connected = self.rh2_controller.is_connected()
            if self.is_connected:
                self.logger.info("RH2控制器连接正常")
            else:
                self.logger.warning("RH2控制器连接异常，尝试重连...")
                try:
                    self.rh2_controller.connect()
                    self.is_connected = self.rh2_controller.is_connected()
                except Exception as e:
                    self.logger.error(f"重连失败: {e}")
        else:
            self.is_connected = False
    
    def command_callback(self, msg: JointState):
        """
        处理控制指令回调
        
        JointState消息格式:
        - name[0]: 命令类型 ("get_motors_info" 或 "move_motors")
        - position: 目标位置列表 (move_motors时使用)
        - velocity: 速度列表 (move_motors时使用) 
        - effort: 电流限制列表 (move_motors时使用)
        """
        try:
            if not self.is_connected:
                self.logger.error("控制器未连接，忽略指令")
                self.publish_error_response("控制器未连接")
                return
            
            if len(msg.name) == 0:
                self.logger.error("无效的指令格式：缺少命令类型")
                self.publish_error_response("无效的指令格式")
                return
            
            command_type = msg.name[0]
            self.logger.debug(f"收到指令: {command_type}")
            
            if command_type == "get_motors_info":
                self.handle_get_motors_info()
                
            elif command_type == "move_motors":
                self.handle_move_motors(msg)
                
            else:
                self.logger.warning(f"未知指令类型: {command_type}")
                self.publish_error_response(f"未知指令类型: {command_type}")
                
        except Exception as e:
            self.logger.error(f"处理指令时发生异常: {e}")
            self.publish_error_response(f"处理指令异常: {e}")
    
    def handle_get_motors_info(self):
        """处理获取电机信息指令"""
        try:
            self.logger.debug("执行获取电机信息")
            motor_info = self.rh2_controller.get_motors_info()
            
            if motor_info:
                self.last_motor_info = motor_info
                self.publish_motor_info_response(motor_info, "get_motors_info")
                self.logger.debug("电机信息获取成功")
            else:
                self.logger.warning("电机信息获取失败")
                self.publish_error_response("电机信息获取失败")
                
        except Exception as e:
            self.logger.error(f"获取电机信息异常: {e}")
            self.publish_error_response(f"获取电机信息异常: {e}")
    
    def handle_move_motors(self, msg: JointState):
        """处理移动电机指令"""
        try:
            # 验证数据长度
            motor_count = len(self.rh2_controller.motor_ids)
            
            if not (len(msg.position) == len(msg.velocity) == len(msg.effort) == motor_count):
                error_msg = f"参数长度不匹配: 需要{motor_count}个参数"
                self.logger.error(error_msg)
                self.publish_error_response(error_msg)
                return
            
            # 转换为整数
            positions = [int(p) for p in msg.position]
            speeds = [int(v) for v in msg.velocity] 
            current_limits = [int(e) for e in msg.effort]
            
            self.logger.debug(f"执行电机移动: 位置={positions}, 速度={speeds}, 电流={current_limits}")
            
            # 执行移动指令
            move_result = self.rh2_controller.move_motors(positions, speeds, current_limits)
            
            if move_result:
                self.last_motor_info = move_result
                self.publish_motor_info_response(move_result, "move_motors")
                self.logger.debug("电机移动指令执行成功")
            else:
                self.logger.warning("电机移动指令执行失败")
                self.publish_error_response("电机移动指令执行失败")
                
        except Exception as e:
            self.logger.error(f"移动电机异常: {e}")
            self.publish_error_response(f"移动电机异常: {e}")
    
    def publish_motor_info_response(self, motor_info: Dict[int, Dict], command_type: str):
        """发布电机信息响应"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "rh2_motors"
            
            # 设置关节名称
            msg.name = [f"motor_{motor_id}" for motor_id in sorted(motor_info.keys())]
            msg.name.insert(0, command_type)  # 第一个元素为命令类型
            
            # 提取位置、速度、电流数据
            positions = []
            velocities = []
            efforts = []
            
            for motor_id in sorted(motor_info.keys()):
                info = motor_info[motor_id]
                if 'error' not in info:
                    positions.append(float(info.get('current_position', 0)))
                    velocities.append(float(info.get('current_speed', 0)))
                    efforts.append(float(info.get('current_current', 0)))
                else:
                    # 错误情况下填充NaN
                    positions.append(float('nan'))
                    velocities.append(float('nan')) 
                    efforts.append(float('nan'))
            
            msg.position = positions
            msg.velocity = velocities
            msg.effort = efforts
            
            self.response_publisher.publish(msg)
            self.logger.debug(f"发布响应: {command_type}")
            
        except Exception as e:
            self.logger.error(f"发布响应失败: {e}")
    
    def publish_error_response(self, error_message: str):
        """发布错误响应"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "rh2_motors"
            
            msg.name = ["error", error_message]
            msg.position = []
            msg.velocity = []
            msg.effort = []
            
            self.response_publisher.publish(msg)
            self.logger.debug(f"发布错误响应: {error_message}")
            
        except Exception as e:
            self.logger.error(f"发布错误响应失败: {e}")
    
    def publish_status(self):
        """定期发布状态信息"""
        if not self.is_connected:
            self.check_connection()
            return
        
        # 定期获取电机状态 (降低频率避免过载)
        current_time = time.time()
        if not hasattr(self, '_last_status_time') or current_time - self._last_status_time > 1.0:
            self._last_status_time = current_time
            try:
                motor_info = self.rh2_controller.get_motors_info()
                if motor_info:
                    self.last_motor_info = motor_info
                    self.publish_motor_info_response(motor_info, "status_update")
            except Exception as e:
                self.logger.debug(f"状态更新失败: {e}")
    
    def get_motor_status_dict(self) -> Dict:
        """获取当前电机状态字典格式"""
        return self.last_motor_info
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def shutdown(self):
        """关闭包装器"""
        self.logger.info("关闭RH2 ROS包装器")
        if self.rh2_controller:
            self.rh2_controller.disconnect()
        self.destroy_node()


def create_command_message(command_type: str, positions: List[int] = None, 
                          speeds: List[int] = None, current_limits: List[int] = None) -> JointState:
    """
    创建控制指令消息的辅助函数
    
    Args:
        command_type: "get_motors_info" 或 "move_motors"
        positions: 目标位置列表 (move_motors时需要)
        speeds: 速度列表 (move_motors时需要)
        current_limits: 电流限制列表 (move_motors时需要)
    
    Returns:
        JointState消息
    """
    msg = JointState()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.header.frame_id = "rh2_command"
    
    msg.name = [command_type]
    
    if command_type == "move_motors" and positions and speeds and current_limits:
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(s) for s in speeds]
        msg.effort = [float(c) for c in current_limits]
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
        rh2_wrapper = RH2ROSWrapper(
            interface='pcan',
            channel='PCAN_USBBUS1', 
            bitrate=1000000,
            motor_ids=[1, 2, 3, 4, 5, 6],
            node_name='rh2_controller_node'
        )
        
        print("RH2 ROS包装器已启动")
        print("Topics:")
        print("  订阅: /rh2/controller (JointState)")
        print("  发布: /rh2/response (JointState)")
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
