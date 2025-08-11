#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Float64MultiArray
import json
import time
import logging


class RH2V2TestClient(Node):
    def __init__(self):
        super().__init__('rh2_v2_test_client')
        
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
            String,
            control_topic,
            qos_profile
        )
        
        response_topic = f'/rh2/{self.wrapper_name}/response'
        self.response_subscriber = self.create_subscription(
            String,
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
        self.last_responses = []
        self.last_tactile_data = None
        
        print("[测试客户端] RH2 V2测试客户端初始化完成")
        print(f"[测试客户端] 控制话题: {control_topic}")
        print(f"[测试客户端] 响应话题: {response_topic}")
        print(f"[测试客户端] 触觉话题: {tactile_topic}")
        
        # 创建测试定时器
        self.test_timer = self.create_timer(3.0, self.run_test_sequence)
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        self.test_step = 0
    
    def response_callback(self, msg: String):
        self.response_count += 1
        try:
            response_data = json.loads(msg.data)
            self.last_responses.append({
                'timestamp': time.time(),
                'data': response_data
            })
            
            # 只保留最近10个响应
            if len(self.last_responses) > 10:
                self.last_responses.pop(0)
            
            print(f"[响应] 收到响应 (第{self.response_count}次):")
            print(f"  命令ID: {response_data.get('command_id')}")
            print(f"  响应类型: {response_data.get('response_type')}")
            print(f"  成功: {response_data.get('success')}")
            
            if response_data.get('success'):
                data = response_data.get('data', {})
                if 'motor_info' in data:
                    motor_info = data['motor_info']
                    if isinstance(motor_info, dict):
                        print(f"  电机信息: {len(motor_info)}个电机")
                        for motor_id, info in motor_info.items():
                            if isinstance(info, dict) and 'current_position' in info:
                                print(f"    电机{motor_id}: 位置={info.get('current_position', 'N/A')}")
                    else:
                        print(f"  单个电机信息: {motor_info}")
                elif 'move_result' in data:
                    print(f"  移动结果: {data['move_result']}")
            else:
                print(f"  错误: {response_data.get('error')}")
                
        except json.JSONDecodeError as e:
            print(f"[响应] JSON解析失败: {e}")
        except Exception as e:
            print(f"[响应] 处理响应异常: {e}")
    
    def tactile_callback(self, msg: Float64MultiArray):
        self.tactile_count += 1
        self.last_tactile_data = {
            'timestamp': time.time(),
            'total_sensors': len(msg.data),
            'layout': msg.layout,
            'data_preview': msg.data[:16] if len(msg.data) > 16 else msg.data  # 前16个值预览
        }
        
        if self.tactile_count <= 3 or self.tactile_count % 50 == 0:  # 前3次或每50次打印
            print(f"[触觉] 收到触觉数据 (第{self.tactile_count}次):")
            print(f"  总传感器数: {len(msg.data)}")
            print(f"  数据预览: {[f'{v:.1f}' for v in msg.data[:8]]}")
            
            # 解析维度信息
            if len(msg.layout.dim) >= 2:
                motor_count = msg.layout.dim[0].size
                sensors_per_motor = msg.layout.dim[1].size
                print(f"  电机数量: {motor_count}, 每电机传感器数: {sensors_per_motor}")
    
    def create_command(self, command_type: str, **kwargs) -> String:
        """创建命令消息"""
        command_data = {
            "command_id": f"test_{command_type}_{int(time.time()*1000)}",
            "timestamp": time.time(),
            "command_type": command_type
        }
        command_data.update(kwargs)
        
        msg = String()
        msg.data = json.dumps(command_data, ensure_ascii=False)
        return msg
    
    def run_test_sequence(self):
        print(f"\n[测试] === 测试步骤 {self.test_step + 1} ===")
        
        if self.test_step == 0:
            print("[测试] 获取所有电机信息")
            cmd = self.create_command("get_all_motor_info")
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 1:
            print("[测试] 获取单个电机信息 (电机1)")
            cmd = self.create_command("get_motor_info", motor_id=1)
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 2:
            print("[测试] 移动单个电机 (电机1)")
            cmd = self.create_command(
                "move_motor", 
                motor_id=1,
                parameters={
                    "position": 1000,
                    "speed": 500,
                    "current_limit": 300
                }
            )
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 3:
            print("[测试] 移动所有电机")
            cmd = self.create_command(
                "move_all_motors",
                parameters={
                    "positions": [500, -500, 1000, -1000, 800, -800],
                    "speeds": [400, 500, 600, 700, 800, 900],
                    "current_limits": [200, 250, 300, 350, 400, 450]
                }
            )
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 4:
            print("[测试] 复位所有电机到0位置")
            cmd = self.create_command(
                "move_all_motors",
                parameters={
                    "positions": [0, 0, 0, 0, 0, 0],
                    "speeds": [1000, 1000, 1000, 1000, 1000, 1000],
                    "current_limits": [500, 500, 500, 500, 500, 500]
                }
            )
            self.control_publisher.publish(cmd)
            
        elif self.test_step == 5:
            print("[测试] 发送无效命令 (测试错误处理)")
            cmd = self.create_command("invalid_command", test_param="test_value")
            self.control_publisher.publish(cmd)
        
        elif self.test_step == 6:
            print("[测试] 发送格式错误的JSON")
            invalid_msg = String()
            invalid_msg.data = "{ invalid json format"
            self.control_publisher.publish(invalid_msg)
        
        self.test_step = (self.test_step + 1) % 7  # 循环测试
    
    def print_stats(self):
        print(f"\n[统计] === 接收统计 ===")
        print(f"[统计] 响应接收次数: {self.response_count}")
        print(f"[统计] 触觉数据接收次数: {self.tactile_count}")
        
        if self.last_responses:
            print(f"[统计] 最近响应:")
            for i, resp in enumerate(self.last_responses[-3:]):  # 显示最近3个
                data = resp['data']
                age = time.time() - resp['timestamp']
                print(f"  {i+1}. {data.get('response_type')} (ID: {data.get('command_id')}) - {age:.1f}秒前")
        
        if self.last_tactile_data:
            age = time.time() - self.last_tactile_data['timestamp']
            print(f"[统计] 最新触觉数据: {self.last_tactile_data['total_sensors']}个传感器 - {age:.1f}秒前")
        
        print()


def main(args=None):
    rclpy.init(args=args)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        test_client = RH2V2TestClient()
        
        print("RH2 V2测试客户端已启动")
        print("将自动测试新的统一接口...")
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
