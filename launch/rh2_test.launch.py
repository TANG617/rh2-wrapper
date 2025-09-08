#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成RH2测试启动配置"""
    
    # 声明启动参数
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='pcan',
        description='CAN接口类型 (pcan, socketcan等)'
    )
    
    channel_arg = DeclareLaunchArgument(
        'channel', 
        default_value='PCAN_USBBUS1',
        description='CAN通道名称'
    )
    
    hand_name_arg = DeclareLaunchArgument(
        'hand_name',
        default_value='right',
        description='手的名称 (left, right)'
    )
    
    # RH2控制器节点
    rh2_controller_node = Node(
        package='rh2_controller',
        executable='rh2_controller_node',
        name='rh2_controller',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'channel': LaunchConfiguration('channel'),
            'hand_name': LaunchConfiguration('hand_name'),
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 延迟启动测试客户端（等待控制器启动）
    rh2_test_client = TimerAction(
        period=3.0,  # 3秒后启动
        actions=[
            Node(
                package='rh2_controller',
                executable='rh2_test_client',
                name='rh2_test_client',
                output='screen',
                emulate_tty=True
            )
        ]
    )
    
    return LaunchDescription([
        interface_arg,
        channel_arg,
        hand_name_arg,
        rh2_controller_node,
        rh2_test_client
    ])
