import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rh2_manager import RH2ControllerManager
from can_controller import CANController

import time
import logging

logging.basicConfig(level=logging.ERROR)


def display_finger_sensors(finger_data):
    print("\033[2J\033[H", end="")
    
    print("=== 五根手指触觉传感器实时数据（8个传感器/手指）===")
    print(f"更新时间: {time.strftime('%H:%M:%S')}")
    print()
    
    print("手指ID |  传感器0  传感器1  传感器2  传感器3  传感器4  传感器5  传感器6  传感器7")
    print("-" * 85)
    
    for finger_id in sorted(finger_data.keys()):
        sensors = finger_data[finger_id]
        sensor_str = "  ".join(f"{s:6d}" for s in sensors)
        print(f"手指{finger_id}  |  {sensor_str}")

def test_tactile():
    
    can_controller = CANController(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    
    manager = RH2ControllerManager(can_controller=can_controller, motor_ids=[2, 3, 4, 5, 6])
    
    if not manager.is_connected():
        print("连接失败，退出测试")
        return
    
    try:
        while True:
            finger_data = manager.get_tactile_data()
            display_finger_sensors(finger_data)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n程序结束")
    finally:
        manager.disconnect()

if __name__ == "__main__":
    test_tactile()