from ..rh2_controller import RH2Controller
import time
import logging

# 配置日志 - 简化版本
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


def test_controller():
    print("=== RH2控制器测试 ===")
    
    with RH2Controller(motor_ids=[3, 4, 5, 6]) as controller:
        if not controller.is_connected():
            print("连接失败，退出测试")
            return
        
        print("\n1. 批量获取电机初始信息 (0xA0):")
        motors_info = controller.get_motors_info()
        
        time.sleep(1)
        
        print("\n2. 批量移动电机到4096位置 (0xAA指令):")
        target_position = 4096
        print(f"   目标位置: {target_position}")
        
        motors_info = controller.move_motors(positions=[target_position] * len(controller.motor_ids), 
                                              speeds=[2000] * len(controller.motor_ids), 
                                              current_limits=[800] * len(controller.motor_ids))
        time.sleep(2)
        
        test_positions = [2000, 2000, 2000, 2000, 2000]
        motors_info = controller.move_motors(
            positions=test_positions,
            speeds=[2000] * len(test_positions),
            current_limits=[800] * len(test_positions)
        )
        
        time.sleep(2)
        
        motors_info = controller.move_motors(positions=[0] * len(controller.motor_ids), 
                                              speeds=[2000] * len(controller.motor_ids), 
                                              current_limits=[800] * len(controller.motor_ids))
        
        time.sleep(2)
        


if __name__ == "__main__":
    test_controller()