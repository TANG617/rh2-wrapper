#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt16MultiArray, Float64MultiArray, MultiArrayDimension
import time
import logging
from typing import Dict, List
from rh2_manager import RH2ControllerManager
from can_controller import CANController


class RH2ROSWrapperV3(Node):
    """
    RH2控制器ROS2包装器 V3
    
    统一的三个话题接口:
    - /rh2/{name}/control (订阅UInt16MultiArray) - 18个uint16值: position[6] + velocity[6] + current[6]
    - /rh2/{name}/response (发布UInt16MultiArray) - 18个uint16值: position[6] + velocity[6] + current[6]
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
            node_name = f'rh2_{name}_wrapper_v3_node'
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
        # 1. 控制指令订阅者 - UInt16MultiArray[18]: [pos1-6, vel1-6, cur1-6]
        control_topic = f'/rh2/{name}/control'
        self.control_subscriber = self.create_subscription(
            UInt16MultiArray,
            control_topic,
            self.control_callback,
            qos_profile
        )
        
        # 2. 响应信息发布者 - UInt16MultiArray[18]: [pos1-6, vel1-6, cur1-6]
        response_topic = f'/rh2/{name}/response'
        self.response_publisher = self.create_publisher(
            UInt16MultiArray,
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
        
        # 当前电机状态缓存
        self.current_positions = [0] * 6  # 6个电机的当前位置
        self.current_velocities = [0] * 6  # 6个电机的当前速度
        self.current_currents = [0] * 6   # 6个电机的当前电流
        
        # 异步状态更新控制
        self.status_update_running = False
        self.last_status_update_time = 0
        self.status_update_interval = 0.2  # 200ms间隔更新状态，避免频繁CAN通信
        
        # 定时器 - 定期发布状态和触觉数据
        self.info_timer = self.create_timer(0.1, self.publish_motor_status)  # 10Hz状态发布
        self.tactile_timer = self.create_timer(0.05, self.publish_tactile_data)  # 20Hz触觉数据
        self.status_update_timer = self.create_timer(0.2, self.update_motor_status_async)  # 5Hz状态更新
        
        # 检查连接状态
        self.check_connection()
        
        self.logger.info(f"RH2 ROS包装器V3启动完成: {name}")
        self.logger.info(f"话题: {control_topic}, {response_topic}, {tactile_topic}")
        
        # 打印数据格式说明
        self.logger.info("数据格式:")
        self.logger.info("  控制/响应: UInt16Array[18] = [position[6], velocity[6], current[6]]")
        self.logger.info("  触觉: Float64Array[48] = 6电机 × 8传感器")
    
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
    
    def control_callback(self, msg: UInt16MultiArray):
        """
        处理控制指令回调
        
        消息格式: UInt16MultiArray[18]
        - data[0-5]:   6个电机的目标位置 (position[6])
        - data[6-11]:  6个电机的目标速度 (velocity[6])  
        - data[12-17]: 6个电机的电流限制 (current[6])
        """
        try:
            if len(msg.data) != 18:
                self.logger.error(f"控制指令数据长度错误: 期望18个uint16值，收到{len(msg.data)}个")
                return
            
            if not self.is_connected:
                self.logger.error("控制器未连接，忽略控制指令")
                return
            
            # 解析控制数据
            positions = list(msg.data[0:6])      # 位置[0-5]
            velocities = list(msg.data[6:12])    # 速度[6-11] 
            currents = list(msg.data[12:18])     # 电流[12-17]
            
            self.logger.info(f"收到控制指令: 位置={positions}, 速度={velocities}, 电流={currents}")
            
            # 执行电机移动
            try:
                move_results = self.rh2_manager.move_all_motors(
                    positions, velocities, currents
                )
                self.logger.info(f"电机移动执行完成: {len(move_results)}个电机")
                
                # 立即更新状态并发布响应
                self.update_motor_status()
                
            except Exception as e:
                self.logger.error(f"电机移动失败: {e}")
                
        except Exception as e:
            self.logger.error(f"处理控制指令异常: {e}")
    
    def update_motor_status_async(self):
        """异步更新电机状态 - 独立定时器调用"""
        # 防止重复执行
        if self.status_update_running:
            self.logger.debug("状态更新正在进行中，跳过本次更新")
            return
            
        if not self.is_connected:
            return
            
        current_time = time.time()
        if current_time - self.last_status_update_time < self.status_update_interval:
            return
            
        self.status_update_running = True
        try:
            # 使用较短的超时时间避免阻塞
            motor_info = self.rh2_manager.get_all_motor_info(timeout=0.3)
            if motor_info:
                self.last_motor_info = motor_info
                self.last_status_update_time = current_time
                
                # 更新当前状态数组
                for i, motor_id in enumerate(self.motor_ids):
                    if motor_id in motor_info and i < 6:
                        info = motor_info[motor_id]
                        if isinstance(info, dict) and 'current_position' in info:
                            self.current_positions[i] = int(info.get('current_position', 0))
                            self.current_velocities[i] = int(info.get('current_speed', 0))
                            self.current_currents[i] = int(info.get('current_current', 0))
                
        except Exception as e:
            self.logger.debug(f"异步状态更新失败: {e}")
        finally:
            self.status_update_running = False
    
    def update_motor_status(self):
        """同步更新电机状态 - 仅在控制指令后立即调用"""
        try:
            if not self.is_connected:
                return
                
            # 使用很短的超时时间，避免阻塞发布
            motor_info = self.rh2_manager.get_all_motor_info(timeout=0.1)
            if motor_info:
                self.last_motor_info = motor_info
                
                # 更新当前状态数组
                for i, motor_id in enumerate(self.motor_ids):
                    if motor_id in motor_info and i < 6:
                        info = motor_info[motor_id]
                        if isinstance(info, dict) and 'current_position' in info:
                            self.current_positions[i] = int(info.get('current_position', 0))
                            self.current_velocities[i] = int(info.get('current_speed', 0))
                            self.current_currents[i] = int(info.get('current_current', 0))
                
        except Exception as e:
            self.logger.debug(f"同步状态更新失败: {e}")
    
    def publish_motor_status(self):
        """定期发布电机状态 - 只发布缓存数据，不进行CAN通信"""
        if not self.is_connected:
            self.check_connection()
            return
        
        try:
            # 创建响应消息 - 直接使用缓存的状态数据
            response_msg = UInt16MultiArray()
            
            # 组装18个uint16值: [position[6], velocity[6], current[6]]
            response_data = []
            response_data.extend(self.current_positions)   # 位置[0-5]
            response_data.extend(self.current_velocities)  # 速度[6-11]
            response_data.extend(self.current_currents)    # 电流[12-17]
            
            response_msg.data = response_data
            
            # 设置数组维度信息
            # 第一个维度：数据类型 (3种: position, velocity, current)
            type_dim = MultiArrayDimension()
            type_dim.label = "data_types"
            type_dim.size = 3
            type_dim.stride = 18
            
            # 第二个维度：电机数量 (6个)
            motor_dim = MultiArrayDimension()
            motor_dim.label = "motors"
            motor_dim.size = 6
            motor_dim.stride = 6
            
            response_msg.layout.dim = [type_dim, motor_dim]
            response_msg.layout.data_offset = 0
            
            # 发布响应
            self.response_publisher.publish(response_msg)
            
            # 记录详细状态 (降低频率)
            if not hasattr(self, '_last_status_log') or time.time() - self._last_status_log > 2.0:
                self._last_status_log = time.time()
                self.logger.info(f"发布电机状态: 位置={self.current_positions}, 速度={self.current_velocities}, 电流={self.current_currents}")
                
        except Exception as e:
            self.logger.debug(f"电机状态发布失败: {e}")
    
    def publish_tactile_data(self):
        """定期发布触觉数据"""
        if not self.is_connected:
            return
        
        try:
            tactile_data = self.rh2_manager.get_tactile_data()
            
            # 将所有电机的触觉数据组合成一个大数组
            # 格式: [motor1_sensor1-8, motor2_sensor1-8, ..., motor6_sensor1-8]
            all_tactile_values = []
            
            for motor_id in sorted(tactile_data.keys()):
                if motor_id in self.motor_ids:  # 只处理配置的电机
                    sensor_values = tactile_data[motor_id]
                    all_tactile_values.extend(sensor_values)
            
            # 创建Float64MultiArray消息
            msg = Float64MultiArray()
            msg.data = [float(val) for val in all_tactile_values]
            
            # 设置数组维度信息
            # 第一个维度：电机数量
            motor_dim = MultiArrayDimension()
            motor_dim.label = "motors"
            motor_dim.size = len(self.motor_ids)
            motor_dim.stride = len(all_tactile_values)
            
            # 第二个维度：每个电机的传感器数量（8个）
            sensor_dim = MultiArrayDimension()
            sensor_dim.label = "sensors_per_motor"
            sensor_dim.size = 8
            sensor_dim.stride = 8
            
            msg.layout.dim = [motor_dim, sensor_dim]
            msg.layout.data_offset = 0
            
            # 发布触觉数据
            self.tactile_publisher.publish(msg)
            
            # 记录详细信息到日志 (降低频率)
            if not hasattr(self, '_last_tactile_log') or time.time() - self._last_tactile_log > 5.0:
                self._last_tactile_log = time.time()
                self.logger.info(f"发布触觉数据: {len(self.motor_ids)}个电机, 总共{len(all_tactile_values)}个传感器值")
                
                # 按电机分组显示
                for i, motor_id in enumerate(sorted(tactile_data.keys())):
                    if motor_id in self.motor_ids:
                        start_idx = i * 8
                        end_idx = start_idx + 8
                        if end_idx <= len(all_tactile_values):
                            values = all_tactile_values[start_idx:end_idx]
                            self.logger.info(f"  电机{motor_id}: {values}")
                    
        except Exception as e:
            self.logger.debug(f"触觉数据发布失败: {e}")
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def shutdown(self):
        """关闭包装器"""
        self.logger.info("关闭RH2 ROS包装器V3")
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
        # 创建RH2 ROS包装器V3节点
        wrapper_name = "rh2_hand"
        motor_ids = [1, 2, 3, 4, 5, 6]
        
        rh2_wrapper = RH2ROSWrapperV3(
            name=wrapper_name,
            interface='pcan',
            channel='PCAN_USBBUS1', 
            bitrate=1000000,
            motor_ids=motor_ids
        )
        
        print(f"RH2 ROS包装器V3已启动: {wrapper_name}")
        print("统一话题接口:")
        print(f"  订阅: /rh2/{wrapper_name}/control (UInt16MultiArray[18]) - position[6]+velocity[6]+current[6]")
        print(f"  发布: /rh2/{wrapper_name}/response (UInt16MultiArray[18]) - position[6]+velocity[6]+current[6]")
        print(f"  发布: /rh2/{wrapper_name}/tactile (Float64MultiArray[48]) - 6电机×8传感器")
        print()
        print("数据格式:")
        print("  控制/响应数组[18]: [pos1,pos2,pos3,pos4,pos5,pos6, vel1,vel2,vel3,vel4,vel5,vel6, cur1,cur2,cur3,cur4,cur5,cur6]")
        print("  触觉数组[48]: [m1s1,m1s2,...,m1s8, m2s1,m2s2,...,m2s8, ..., m6s1,m6s2,...,m6s8]")
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
