#!/usr/bin/env python3
"""
设置灵巧手刚度的脚本
发送指令：0xa6 0x0b 0x00 0x00 0x00 0x00 0x00 0x00
"""

import sys
import os
import logging
import argparse

# 添加父目录到Python路径以导入RH2Controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rh2_controller import RH2Controller

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
log = logging.getLogger(__name__)


def set_rigidity(motor_ids=None, interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000):
    """
    设置灵巧手刚度
    
    Args:
        motor_ids: 电机ID列表，默认为[1,2,3,4,5,6]
        interface: CAN接口类型
        channel: CAN通道
        bitrate: 波特率
    """
    if motor_ids is None:
        motor_ids = [1, 2, 3, 4, 5, 6]
    
    # 刚度设置指令
    COMMAND_SET_RIGIDITY = 0xA3
    rigidity_payload = [0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    try:
        # 创建控制器实例
        controller = RH2Controller(
            interface=interface,
            channel=channel, 
            bitrate=bitrate,
            auto_connect=True,
            motor_ids=motor_ids
        )
        
        if not controller.is_connected():
            log.error("无法连接到CAN总线")
            return False
            
        log.info(f"已连接到CAN总线: {interface}:{channel}")
        log.info(f"准备发送刚度设置指令到电机: {motor_ids}")
        
        success_count = 0
        failed_motors = []
        
        # 向每个电机发送刚度设置指令
        for motor_id in motor_ids:
            log.info(f"向电机 {motor_id} 发送刚度设置指令...")
            
            success = controller._send_command(
                command=COMMAND_SET_RIGIDITY,
                motor_id=motor_id, 
                data_payload=rigidity_payload
            )
            
            if success:
                success_count += 1
                log.info(f"电机 {motor_id} 刚度设置指令发送成功")
            else:
                failed_motors.append(motor_id)
                log.warning(f"电机 {motor_id} 刚度设置指令发送失败")
        
        # 总结结果
        log.info(f"刚度设置完成: 成功 {success_count}/{len(motor_ids)} 个电机")
        
        if failed_motors:
            log.warning(f"失败的电机ID: {failed_motors}")
            return False
        else:
            log.info("所有电机刚度设置成功!")
            return True
            
    except Exception as e:
        log.error(f"设置刚度时发生错误: {e}")
        return False
    finally:
        if 'controller' in locals():
            controller.disconnect()
            log.info("已断开CAN总线连接")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='设置RH2灵巧手刚度')
    parser.add_argument('--motors', '-m', nargs='+', type=int, default=[1,2,3,4,5,6],
                        help='要设置的电机ID列表 (默认: 1 2 3 4 5 6)')
    parser.add_argument('--interface', '-i', default='pcan',
                        help='CAN接口类型 (默认: pcan)')
    parser.add_argument('--channel', '-c', default='PCAN_USBBUS1', 
                        help='CAN通道 (默认: PCAN_USBBUS1)')
    parser.add_argument('--bitrate', '-b', type=int, default=1000000,
                        help='CAN波特率 (默认: 1000000)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='启用详细日志输出')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    log.info("=== RH2 灵巧手刚度设置工具 ===")
    log.info(f"目标电机: {args.motors}")
    log.info(f"CAN配置: {args.interface}:{args.channel}@{args.bitrate}")
    log.info(f"发送指令: 0xA3 0x0C 0x00 0x00 0x00 0x00 0x00 0x00")
    
    success = set_rigidity(
        motor_ids=args.motors,
        interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate
    )
    
    if success:
        log.info("刚度设置完成!")
        sys.exit(0)
    else:
        log.error("刚度设置失败!")
        sys.exit(1)


if __name__ == "__main__":
    main()
