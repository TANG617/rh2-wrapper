#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成RH2控制器启动配置"""
    
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
    
    bitrate_arg = DeclareLaunchArgument(
        'bitrate',
        default_value='1000000',
        description='CAN总线波特率'
    )
    
    motor_ids_arg = DeclareLaunchArgument(
        'motor_ids',
        default_value='[1,2,3,4,5,6]',
        description='电机ID列表'
    )
    
    # RH2控制器节点
    rh2_controller_node = Node(
        package='rh2_controller',
        executable='rh2_controller_node',
        name='rh2_controller',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'channel': LaunchConfiguration('channel'),
            'bitrate': LaunchConfiguration('bitrate'),
            'motor_ids': LaunchConfiguration('motor_ids'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        interface_arg,
        channel_arg, 
        bitrate_arg,
        motor_ids_arg,
        rh2_controller_node
    ])
