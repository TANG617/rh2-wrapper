#!/usr/bin/env python3
"""
RH2硬件测试脚本
测试socketcan连接的真实硬件
"""

import rclpy
import logging
import time
from rh2_ros_wrapper import RH2ROSWrapper

def test_single_hand(hand_name: str, can_channel: str):
    """测试单个手的连接"""
    print(f"\n🤖 测试{hand_name}手 (使用{can_channel})")
    print("=" * 40)
    
    try:
        # 创建控制器
        controller = RH2ROSWrapper(
            interface='socketcan',
            channel=can_channel,
            bitrate=1000000,
            motor_ids=[1, 2, 3, 4, 5, 6],
            hand_name=hand_name,
            node_name=f'rh2_{hand_name}_test'
        )
        
        print(f"✅ {hand_name}手控制器创建成功")
        print(f"📡 连接状态: {controller.get_connection_status()}")
        
        if controller.get_connection_status():
            print(f"📊 正在获取{hand_name}手电机信息...")
            controller.handle_get_motors_info()
            time.sleep(2)
            
            motor_status = controller.get_motor_status_dict()
            if motor_status:
                print(f"🔧 {hand_name}手电机状态:")
                for motor_id, info in motor_status.items():
                    if 'error' not in info:
                        position = info.get('current_position', 'N/A')
                        print(f"   电机{motor_id}: 位置={position}")
                    else:
                        print(f"   电机{motor_id}: 错误 - {info['error']}")
            else:
                print(f"⚠️  {hand_name}手电机信息获取失败")
        else:
            print(f"❌ {hand_name}手连接失败")
        
        # 清理
        controller.shutdown()
        return controller.get_connection_status()
        
    except Exception as e:
        print(f"❌ {hand_name}手测试失败: {e}")
        return False

def test_dual_hands():
    """测试双手控制"""
    print("\n🤖🤖 测试双手控制")
    print("=" * 50)
    
    try:
        from rh2_node import RH2DualHandNode
        
        dual_node = RH2DualHandNode()
        print("✅ 双手节点创建成功")
        
        # 简单测试
        print("📊 检查双手状态...")
        dual_node.check_hands_status()
        
        time.sleep(3)
        
        # 清理
        dual_node.shutdown()
        print("✅ 双手测试完成")
        return True
        
    except Exception as e:
        print(f"❌ 双手测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🚀 RH2硬件连接测试")
    print("📡 使用socketcan接口 (can0/can1)")
    print("=" * 50)
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 测试右手 (can0)
        right_ok = test_single_hand("right", "can0")
        
        # 测试左手 (can1) 
        left_ok = test_single_hand("left", "can1")
        
        print(f"\n📋 测试结果总结:")
        print(f"  右手 (can0): {'✅ 连接正常' if right_ok else '❌ 连接失败'}")
        print(f"  左手 (can1): {'✅ 连接正常' if left_ok else '❌ 连接失败'}")
        
        # 如果都成功，测试双手控制
        if right_ok and left_ok:
            test_dual_hands()
        elif right_ok or left_ok:
            print("\n💡 提示: 只有一只手连接成功，可以进行单手测试")
        else:
            print("\n⚠️  没有检测到硬件连接")
            print("💡 建议:")
            print("  1. 检查CAN设备是否正确连接")
            print("  2. 确认CAN接口配置正确")
            print("  3. 尝试使用模拟模式进行测试: python3 run_sim_node.py")
        
    except KeyboardInterrupt:
        print("\n⏹️  测试被用户中断")
    except Exception as e:
        print(f"❌ 测试过程中出错: {e}")
    finally:
        rclpy.shutdown()
        print("\n✨ 硬件测试结束")

if __name__ == "__main__":
    main()
