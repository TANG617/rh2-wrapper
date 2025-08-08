from rh2_controller import RH2Controller
import time


def test_controller():
    print("=== RH2批量控制器测试 (标准CAN) ===")
    
    with RH2Controller(motor_ids=[2, 3, 4, 5, 6]) as controller:
        if not controller.is_connected():
            print("连接失败，退出测试")
            return
        
        print("连接成功，开始测试...")
        
        print("\n1. 批量获取电机信息 (0xA0):")
        motor_infos = controller.batch_get_motors_info()
        for motor_id, info in motor_infos.items():
            if 'error' not in info:
                print(f"  电机{motor_id}: 位置={info.get('current_position')}, "
                      f"速度={info.get('current_speed')}, 电流={info.get('current_current')}, "
                      f"状态={info.get('status')}")
            else:
                print(f"  电机{motor_id}: {info['error']}")
        
        time.sleep(1)
        
        print("\n2. 移动所有电机到4096位置:")
        target_position = 4096
        print(f"   发送移动指令: 目标位置={target_position}")
        move_results = controller.move_all_motors_to_same_position(target_position, 2000, 800)
        
        for motor_id, result in move_results.items():
            if 'error' not in result:
                print(f"   电机{motor_id}: 移动指令已发送，应答状态={result.get('status')}")
            else:
                print(f"   电机{motor_id}: 发送失败 - {result['error']}")
        
        print("   等待电机运动完成...")
        time.sleep(3)
        
        print(f"\n3. 验证电机位置是否接近目标位置{target_position}:")
        positions_after_move = controller.get_all_positions()
        
        for motor_id, position in positions_after_move.items():
            if position is not None:
                position_error = abs(position - target_position)
                error_percentage = (position_error / target_position) * 100
                
                print(f"   电机{motor_id}: 当前位置={position}, 目标位置={target_position}")
                print(f"   电机{motor_id}: 位置误差={position_error}, 误差百分比={error_percentage:.2f}%")
                
                if position_error <= target_position * 0.05:
                    print(f"   电机{motor_id}: ✅ 位置正确")
                else:
                    print(f"   电机{motor_id}: ❌ 位置误差过大")
            else:
                print(f"   电机{motor_id}: ❌ 无法读取位置")
        
        time.sleep(1)
        
        print("\n4. 获取详细电机信息 (使用C结构体解析):")
        detailed_infos = controller.batch_get_motors_info()
        for motor_id, info in detailed_infos.items():
            if 'error' not in info:
                pos = info.get('current_position', 'N/A')
                speed = info.get('current_speed', 'N/A')
                current = info.get('current_current', 'N/A')
                status = info.get('status', 'N/A')
                status_desc = info.get('status_description', 'N/A')
                
                print(f"   电机{motor_id}: 位置={pos}, 速度={speed}, 电流={current}mA")
                print(f"   电机{motor_id}: 状态=0x{status:02X} ({status_desc})")
                
                c_struct = info.get('c_struct_demo', {})
                if c_struct:
                    print(f"   电机{motor_id}: C结构体 P={c_struct.get('P')}, V={c_struct.get('V')}, I={c_struct.get('I')}, F={c_struct.get('F')}")
                
                force_adc = info.get('force_sensor_adc', 'N/A')
                pos_norm = info.get('position_normalized', 0)
                print(f"   电机{motor_id}: 位置百分比={pos_norm*100:.1f}%, 压力传感器ADC={force_adc}")
                
            else:
                print(f"   电机{motor_id}: {info['error']}")
        
        time.sleep(1)
        
        print("\n5. 回零所有电机:")
        zero_results = controller.move_all_motors_to_same_position(0, 2000, 800)
        for motor_id, result in zero_results.items():
            if 'error' not in result:
                print(f"   电机{motor_id}: 回零指令已发送")
            else:
                print(f"   电机{motor_id}: {result['error']}")
        
        time.sleep(2)
        
        print("\n6. 验证回零位置:")
        zero_positions = controller.get_all_positions()
        for motor_id, position in zero_positions.items():
            if position is not None:
                print(f"   电机{motor_id}: 回零后位置={position}")
                if position <= 100:
                    print(f"   电机{motor_id}: ✅ 回零成功")
                else:
                    print(f"   电机{motor_id}: ❌ 回零不完全")
            else:
                print(f"   电机{motor_id}: ❌ 无法读取回零位置")

if __name__ == "__main__":
    test_controller()