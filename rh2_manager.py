#!/usr/bin/env python3
from typing import Dict, List, Optional
from rh2_controller import RH2Controller
from can_controller import CANController

# , interface: str = 'pcan', channel: str = 'PCAN_USBBUS1',  bitrate: int = 1000000,
class RH2ControllerManager:
    def __init__(self,
                 can_controller:CANController = None,
                 motor_ids: List[int] = [1, 2, 3, 4, 5, 6]):
        if(can_controller == None): raise ValueError
        self.can_controller = can_controller
        self.controllers = {}
        
        for motor_id in motor_ids:
            self.controllers[motor_id] = RH2Controller(motor_id, self.can_controller)
    
    def is_connected(self) -> bool:
        return self.can_controller.is_connected()
    
    def disconnect(self):
        self.can_controller.disconnect()
    
    def get_controller(self, motor_id: int) -> Optional[RH2Controller]:
        """获取指定电机ID的控制器"""
        return self.controllers.get(motor_id)
    
    def get_all_controllers(self) -> Dict[int, RH2Controller]:
        """获取所有控制器"""
        return self.controllers
    
    def get_tactile_data(self) -> Dict[int, List[int]]:
        tactile_data = {}
        
        for motor_id, controller in self.controllers.items():
            tactile_data[motor_id] = controller.get_tactile_data()
        
        return tactile_data
    
    def get_all_motor_info(self, timeout: float = 1.0) -> Dict[int, Dict]:
        results = {}
        
        for motor_id, controller in self.controllers.items():
            results[motor_id] = controller.get_motor_info(timeout)
        
        return results
    
    def move_all_motors(self, positions: List[int], speeds: List[int], 
                       current_limits: List[int], timeout: float = 1.0) -> Dict[int, Dict]:
        results = {}
        motor_ids = list(self.controllers.keys())
        
        if not (len(positions) == len(speeds) == len(current_limits) <= len(motor_ids)):
            return {}
        
        for i, motor_id in enumerate(motor_ids):
            controller = self.controllers[motor_id]
            results[motor_id] = controller.move_motor(positions[i], speeds[i], current_limits[i], timeout)
        
        return results
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
    
    def __del__(self):
        self.disconnect()
