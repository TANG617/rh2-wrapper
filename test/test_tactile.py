import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rh2_controller import RH2Controller

import time
import logging

# 配置日志 - 静默模式，只显示错误
logging.basicConfig(
    level=logging.ERROR,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


def display_finger_sensors(finger_data):
    """实时显示5根手指的传感器数据"""
    # 清屏并移到顶部
    print("\033[2J\033[H", end="")
    
    print("=== 五根手指触觉传感器实时数据（8个传感器/手指）===")
    print(f"更新时间: {time.strftime('%H:%M:%S')}")
    print()
    
    # 表头
    print("手指ID |  传感器0  传感器1  传感器2  传感器3  传感器4  传感器5  传感器6  传感器7")
    print("-" * 85)
    
    # 数据行
    for finger_id in sorted(finger_data.keys()):
        sensors = finger_data[finger_id]
        sensor_str = "  ".join(f"{s:6d}" for s in sensors)
        print(f"手指{finger_id}  |  {sensor_str}")

def test_tactile():
    """高速实时触觉传感器测试"""
    controller = RH2Controller(motor_ids=[ 2, 3, 4, 5, 6])
    
    if not controller.is_connected():
        print("连接失败，退出测试")
        return
    
    try:
        while True:
            # 读取所有手指数据
            finger_data = controller.get_finger_sensors()
            
            # 显示数据
            display_finger_sensors(finger_data)
            
            # 短暂延时以控制刷新率
            time.sleep(0.1)  # 10Hz刷新率
            
    except KeyboardInterrupt:
        print("\n\n程序结束")
    finally:
        controller.disconnect()

if __name__ == "__main__":
    test_tactile()