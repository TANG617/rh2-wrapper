import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
        
        # 测试每个电机的多帧触觉数据读取
        for motor_id in controller.motor_ids:
            print(f"\n--- 测试电机{motor_id}的多帧触觉数据 ---")
            
            result = controller.get_tactile_data_frames(motor_id, timeout=3.0)
            
            if result.get('success'):
                print(f"✓ 电机{motor_id}读取成功，共{result['total_frames']}帧")
                
                # 显示所有原始数据
                print("所有原始数据:")
                for frame_num, frame_data in result['frames'].items():
                    raw_hex = frame_data['raw_data']
                    raw_bytes = frame_data['raw_bytes']
                    tactile_data = frame_data['tactile_data']
                    is_last = frame_data['is_last_frame']
                    
                    print(f"  帧{frame_num}: {raw_hex} (字节: {raw_bytes})")
                    print(f"    触觉数据: {tactile_data}, 最后一帧: {is_last}")
                
            else:
                print(f"✗ 电机{motor_id}读取失败: {result.get('error', '未知错误')}")
                if 'frames' in result and result['frames']:
                    print(f"  部分数据（{len(result['frames'])}帧）:")
                    for frame_num, frame_data in result['frames'].items():
                        print(f"    帧{frame_num}: {frame_data['raw_data']}")
        


if __name__ == "__main__":
    test_tactile()