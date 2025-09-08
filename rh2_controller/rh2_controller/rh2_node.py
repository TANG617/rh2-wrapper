#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import logging
from .rh2_ros_wrapper import RH2ROSWrapper

class RH2DualHandNode(Node):
    """
    RH2双手控制节点
    
    功能:
    - 同时创建和管理左右手的RH2ROSWrapper实例
    - 分别使用PCAN_USBBUS1和PCAN_USBBUS2控制左右手
    - 提供统一的启动和关闭接口
    
    Topics:
    左手:
    - /ry_hand/left/set_angles (订阅)
    - /ry_hand/left/joint_states (发布)
    右手:
    - /ry_hand/right/set_angles (订阅)
    - /ry_hand/right/joint_states (发布)
    """
    
    def __init__(self):
        super().__init__('rh2_dual_hand_node')
        
        # 配置日志
        self.logger = self.get_logger()
        
        try:
            # 创建右手控制器
            self.right_hand = RH2ROSWrapper(
                interface='pcan',
                channel='PCAN_USBBUS1',
                bitrate=1000000,
                motor_ids=[1, 2, 3, 4, 5, 6],
                hand_name='right',
                node_name='rh2_right_hand'
            )
            self.logger.info("右手控制器初始化成功")
            
            # 创建左手控制器
            self.left_hand = RH2ROSWrapper(
                interface='pcan',
                channel='PCAN_USBBUS2',
                bitrate=1000000,
                motor_ids=[1, 2, 3, 4, 5, 6],
                hand_name='left',
                node_name='rh2_left_hand'
            )
            self.logger.info("左手控制器初始化成功")
            
        except Exception as e:
            self.logger.error(f"初始化双手控制器失败: {e}")
            raise
        
        # 创建状态检查定时器
        self.create_timer(1.0, self.check_hands_status)  # 每秒检查一次状态
        
        self.logger.info("RH2双手控制节点初始化完成")
    
    def check_hands_status(self):
        """检查双手状态"""
        right_status = "已连接" if self.right_hand.get_connection_status() else "未连接"
        left_status = "已连接" if self.left_hand.get_connection_status() else "未连接"
        
        self.logger.debug(f"手部状态 - 右手: {right_status}, 左手: {left_status}")
    
    def shutdown(self):
        """关闭节点"""
        self.logger.info("正在关闭RH2双手控制节点...")
        
        try:
            if hasattr(self, 'right_hand'):
                self.right_hand.shutdown()
                self.logger.info("右手控制器已关闭")
            
            if hasattr(self, 'left_hand'):
                self.left_hand.shutdown()
                self.logger.info("左手控制器已关闭")
                
        except Exception as e:
            self.logger.error(f"关闭过程中发生错误: {e}")
        finally:
            self.destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # 创建双手控制节点
        rh2_node = RH2DualHandNode()
        
        print("\nRH2双手控制节点已启动")
        print("Topics:")
        print("右手:")
        print("  订阅: /ry_hand/right/set_angles (JointState)")
        print("  发布: /ry_hand/right/joint_states (JointState)")
        print("左手:")
        print("  订阅: /ry_hand/left/set_angles (JointState)")
        print("  发布: /ry_hand/left/joint_states (JointState)")
        print("\n按 Ctrl+C 退出")
        
        # 运行节点
        rclpy.spin(rh2_node)
        
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭...")
    except Exception as e:
        print(f"运行时错误: {e}")
    finally:
        if 'rh2_node' in locals():
            rh2_node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
