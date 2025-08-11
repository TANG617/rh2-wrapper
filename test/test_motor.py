import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rh2_manager import RH2ControllerManager
from can_controller import CANController

import time
import logging

# 配置日志 - 简化版本
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


def test_controller():
    print("=== RH2控制器测试 ===")
    
    # 创建CAN控制器
    can_controller = CANController(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    
    # 创建管理器
    manager = RH2ControllerManager(can_controller=can_controller, motor_ids=[3, 4, 5, 6])
    
    if not manager.is_connected():
        print("连接失败，退出测试")
        return
    
    try:
        print("\n1. 批量获取电机初始信息 (0xA0):")
        motors_info = manager.get_all_motor_info()
        for motor_id, info in motors_info.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                print(f"   电机{motor_id}: 当前位置={pos}")
            else:
                print(f"   电机{motor_id}: {info['error']}")
        
        time.sleep(1)
        
        print("\n2. 批量移动电机到4096位置 (0xAA指令):")
        target_position = 4096
        print(f"   目标位置: {target_position}")
        
        motor_ids = list(manager.controllers.keys())
        motors_info = manager.move_all_motors(
            positions=[target_position] * len(motor_ids), 
            speeds=[2000] * len(motor_ids), 
            current_limits=[800] * len(motor_ids)
        )
        
        for motor_id, info in motors_info.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                print(f"   电机{motor_id}: 移动后位置={pos}")
        
        time.sleep(2)
        
        print("\n3. 移动到中间位置2000:")
        test_positions = [2000] * len(motor_ids)
        motors_info = manager.move_all_motors(
            positions=test_positions,
            speeds=[2000] * len(test_positions),
            current_limits=[800] * len(test_positions)
        )
        
        for motor_id, info in motors_info.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                print(f"   电机{motor_id}: 移动后位置={pos}")
        
        time.sleep(2)
        
        print("\n4. 回到原点位置0:")
        motors_info = manager.move_all_motors(
            positions=[0] * len(motor_ids), 
            speeds=[2000] * len(motor_ids), 
            current_limits=[800] * len(motor_ids)
        )
        
        for motor_id, info in motors_info.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                print(f"   电机{motor_id}: 回到位置={pos}")
        
        time.sleep(2)
        
        print("\n5. 测试单个电机控制:")
        # 获取单个控制器进行个别操作
        motor_3 = manager.get_controller(3)
        if motor_3:
            print("   使用单个控制器移动电机3到位置1500:")
            result = motor_3.move_motor(position=1500, speed=1000, current_limit=600)
            if 'error' not in result:
                pos = result.get('current_position', 'N/A')
                print(f"   电机3: 单独移动后位置={pos}")
            else:
                print(f"   电机3: {result['error']}")
        
        print("\n测试完成!")
        
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        manager.disconnect()
        


if __name__ == "__main__":
    test_controller()