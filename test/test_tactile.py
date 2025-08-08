from rh2_controller import RH2Controller
import time
import logging

# 配置日志 - 简化版本
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


def test_tactile():
    print("=== 触觉测试 ===")
    
    with RH2Controller(motor_ids=[1,2, 3, 4, 5, 6]) as controller:
        if not controller.is_connected():
            print("连接失败，退出测试")
            return
        
        tactile_info = controller.get_tactile_info()
        for tactile_frame_id, info in tactile_info.items():
            tactile_values = info.get('tactile_values', 'N/A')
            print(f"电机{tactile_frame_id}: 触觉数据={tactile_values}")
        


if __name__ == "__main__":
    test_tactile()