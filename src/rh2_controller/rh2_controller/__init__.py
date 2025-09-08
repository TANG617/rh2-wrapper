"""
RH2机器人手控制器ROS2包

这个包提供了RH2机器人手的ROS2接口，支持：
- 双手控制
- 电机位置、速度和电流控制
- 实时状态反馈
"""

from .rh2_controller import RH2Controller
from .rh2_ros_wrapper import RH2ROSWrapper
from .rh2_node import RH2DualHandNode
