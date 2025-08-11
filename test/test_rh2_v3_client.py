#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt16MultiArray, Float64MultiArray, MultiArrayDimension
import time
import logging


class RH2V3TestClient(Node):
    def __init__(self):
        super().__init__('rh2_v3_test_client')
        
        self.logger = self.get_logger()
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.wrapper_name = "rh2_hand"
        
        # 创建发布者和订阅者
        control_topic = f'/rh2/{self.wrapper_name}/control'
        self.control_publisher = self.create_publisher(
            UInt16MultiArray,
            control_topic,
            qos_profile
        )
        
        response_topic = f'/rh2/{self.wrapper_name}/response'
        self.response_subscriber = self.create_subscription(
            UInt16MultiArray,
            response_topic,
            self.response_callback,
            qos_profile
        )
        
        tactile_topic = f'/rh2/{self.wrapper_name}/tactile'
        self.tactile_subscriber = self.create_subscription(
            Float64MultiArray,
            tactile_topic,
            self.tactile_callback,
            qos_profile
        )
        
        # 统计信息
        self.response_count = 0
        self.tactile_count = 0
        self.last_response_data = None
        self.last_tactile_data = None
        
        print("[测试客户端] RH2 V3测试客户端初始化完成")
        print(f"[测试客户端] 控制话题: {control_topic}")
        print(f"[测试客户端] 响应话题: {response_topic}")
        print(f"[测试客户端] 触觉话题: {tactile_topic}")
        print()
        print("数据格式:")
        print("  控制/响应: UInt16Array[18] = [position[6], velocity[6], current[6]]")
        print("  触觉: Float64Array[48] = 6电机 × 8传感器")
        
        # 创建测试定时器
        self.test_timer = self.create_timer(4.0, self.run_test_sequence)
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        self.test_step = 0
    
    def response_callback(self, msg: UInt16MultiArray):
        self.response_count += 1
        
        if len(msg.data) != 18:
            print(f"[响应] 警告: 数据长度错误，期望18个值，收到{len(msg.data)}个")
            return
        
        # 解析响应数据
        positions = list(msg.data[0:6])      # 位置[0-5]
        velocities = list(msg.data[6:12])    # 速度[6-11]
        currents = list(msg.data[12:18])     # 电流[12-17]
        
        self.last_response_data = {
            'timestamp': time.time(),
            'positions': positions,
            'velocities': velocities,
            'currents': currents
        }
        
        if self.response_count <= 5 or self.response_count % 50 == 0:  # 前5次或每50次打印
            print(f"[响应] 收到响应数据 (第{self.response_count}次):")
            print(f"  位置: {positions}")
            print(f"  速度: {velocities}")
            print(f"  电流: {currents}")
    
    def tactile_callback(self, msg: Float64MultiArray):
        self.tactile_count += 1
        
        expected_length = 48  # 6电机 × 8传感器
        if len(msg.data) != expected_length:
            print(f"[触觉] 警告: 数据长度错误，期望{expected_length}个值，收到{len(msg.data)}个")
            return
        
        self.last_tactile_data = {
            'timestamp': time.time(),
            'total_sensors': len(msg.data),
            'data_preview': msg.data[:16] if len(msg.data) > 16 else msg.data
        }
        
        if self.tactile_count <= 3 or self.tactile_count % 100 == 0:  # 前3次或每100次打印
            print(f"[触觉] 收到触觉数据 (第{self.tactile_count}次):")
            print(f"  总传感器数: {len(msg.data)}")
            
            # 按电机分组显示前两个电机的数据
            for motor_idx in range(min(2, 6)):  # 只显示前两个电机
                start_idx = motor_idx * 8
                end_idx = start_idx + 8
                motor_data = msg.data[start_idx:end_idx]
                print(f"  电机{motor_idx+1}: {[f'{v:.1f}' for v in motor_data]}")
    
    def create_control_command(self, positions, velocities, currents) -> UInt16MultiArray:
        """
        创建控制命令消息
        
        Args:
            positions: 6个电机的目标位置 [pos1, pos2, pos3, pos4, pos5, pos6]
            velocities: 6个电机的目标速度 [vel1, vel2, vel3, vel4, vel5, vel6]
            currents: 6个电机的电流限制 [cur1, cur2, cur3, cur4, cur5, cur6]
        """
        msg = UInt16MultiArray()
        
        # 组装18个uint16值: [position[6], velocity[6], current[6]]
        control_data = []
        control_data.extend(positions)   # 位置[0-5]
        control_data.extend(velocities)  # 速度[6-11]
        control_data.extend(currents)    # 电流[12-17]
        
        msg.data = control_data
        
        # 设置数组维度信息
        type_dim = MultiArrayDimension()
        type_dim.label = "data_types"
        type_dim.size = 3
        type_dim.stride = 18
        
        motor_dim = MultiArrayDimension()
        motor_dim.label = "motors"
        motor_dim.size = 6
        motor_dim.stride = 6
        
        msg.layout.dim = [type_dim, motor_dim]
        msg.layout.data_offset = 0
        
        return msg
    
    def run_test_sequence(self):
        print(f"\n[测试] === 测试步骤 {self.test_step + 1} ===")
        
        if self.test_step == 0:
            print("[测试] 发送零位置命令")
            positions = [0, 0, 0, 0, 0, 0]
            velocities = [500, 500, 500, 500, 500, 500]
            currents = [200, 200, 200, 200, 200, 200]
            cmd = self.create_control_command(positions, velocities, currents)
            print(f"  位置: {positions}")
            print(f"  速度: {velocities}")
            print(f"  电流: {currents}")
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 1:
            print("[测试] 发送正向移动命令")
            positions = [1000, 500, 1500, 800, 1200, 600]
            velocities = [400, 600, 500, 700, 450, 550]
            currents = [300, 250, 350, 280, 320, 270]
            cmd = self.create_control_command(positions, velocities, currents)
            print(f"  位置: {positions}")
            print(f"  速度: {velocities}")
            print(f"  电流: {currents}")
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 2:
            print("[测试] 发送负向移动命令")
            positions = [65535-500, 65535-1000, 65535-800, 65535-1200, 65535-600, 65535-900]  # 使用补码表示负值
            velocities = [600, 800, 700, 900, 650, 750]
            currents = [400, 350, 450, 380, 420, 370]
            cmd = self.create_control_command(positions, velocities, currents)
            print(f"  位置: {positions}")
            print(f"  速度: {velocities}")
            print(f"  电流: {currents}")
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 3:
            print("[测试] 发送复位命令")
            positions = [0, 0, 0, 0, 0, 0]
            velocities = [1000, 1000, 1000, 1000, 1000, 1000]
            currents = [500, 500, 500, 500, 500, 500]
            cmd = self.create_control_command(positions, velocities, currents)
            print(f"  位置: {positions}")
            print(f"  速度: {velocities}")
            print(f"  电流: {currents}")
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 4:
            print("[测试] 发送交替移动命令")
            positions = [800, 0, 1200, 0, 1000, 0]
            velocities = [500, 500, 600, 600, 550, 550]
            currents = [300, 300, 350, 350, 325, 325]
            cmd = self.create_control_command(positions, velocities, currents)
            print(f"  位置: {positions}")
            print(f"  速度: {velocities}")
            print(f"  电流: {currents}")
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 5:
            print("[测试] 发送最大速度测试")
            positions = [2000, 1500, 2500, 1800, 2200, 1600]
            velocities = [2000, 2000, 2000, 2000, 2000, 2000]
            currents = [800, 800, 800, 800, 800, 800]
            cmd = self.create_control_command(positions, velocities, currents)
            print(f"  位置: {positions}")
            print(f"  速度: {velocities}")
            print(f"  电流: {currents}")
            self.control_publisher.publish(cmd)
        
        elif self.test_step == 6:
            print("[测试] 发送错误数据长度测试")
            msg = UInt16MultiArray()
            msg.data = [100, 200, 300]  # 故意发送错误长度的数据
            print(f"  发送{len(msg.data)}个值 (应该是18个)")
            self.control_publisher.publish(msg)
        
        self.test_step = (self.test_step + 1) % 7  # 循环测试
    
    def print_stats(self):
        print(f"\n[统计] === 接收统计 ===")
        print(f"[统计] 响应接收次数: {self.response_count}")
        print(f"[统计] 触觉数据接收次数: {self.tactile_count}")
        
        if self.last_response_data:
            age = time.time() - self.last_response_data['timestamp']
            print(f"[统计] 最新响应数据 ({age:.1f}秒前):")
            print(f"  位置: {self.last_response_data['positions']}")
            print(f"  速度: {self.last_response_data['velocities']}")
            print(f"  电流: {self.last_response_data['currents']}")
        
        if self.last_tactile_data:
            age = time.time() - self.last_tactile_data['timestamp']
            print(f"[统计] 最新触觉数据: {self.last_tactile_data['total_sensors']}个传感器 ({age:.1f}秒前)")
        
        print()


def main(args=None):
    rclpy.init(args=args)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        test_client = RH2V3TestClient()
        
        print("RH2 V3测试客户端已启动")
        print("将自动测试新的UInt16数组接口...")
        print("按 Ctrl+C 退出")
        
        rclpy.spin(test_client)
        
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭...")
    except Exception as e:
        print(f"运行时错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'test_client' in locals():
            test_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
