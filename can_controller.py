#!/usr/bin/env python3
import can
import time
import struct
import logging
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class CANController:
    def __init__(self, interface: str = 'pcan', channel: str = 'PCAN_USBBUS1', 
                 bitrate: int = 1000000, auto_connect: bool = True):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.connected = False
        
        if auto_connect:
            self.connect()
    
    def connect(self) -> bool:
        try:
            bus_kwargs = {
                "interface": self.interface,
                "channel": self.channel,
                "bitrate": self.bitrate,
                "state": can.BusState.ACTIVE,
            }
            self.bus = can.Bus(**bus_kwargs)
            self.connected = True
            return True
        except Exception as e:
            self.connected = False
            return False
    
    def disconnect(self):
        if self.bus and self.connected:
            self.bus.shutdown()
            self.connected = False
    
    def is_connected(self) -> bool:
        return self.connected
    
    def send_message(self, arbitration_id: int, data: List[int]) -> bool:
        if not self.connected:
            return False
        
        while len(data) < 8:
            data.append(0x00)
        
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def receive_message(self, timeout: float = 0.1) -> Optional[can.Message]:
        if not self.connected:
            return None
        
        try:
            return self.bus.recv(timeout=timeout)
        except:
            return None
    
    def collect_responses(self, expected_ids: List[int], timeout: float = 1.0) -> Dict[int, Optional[can.Message]]:
        results = {}
        expected_response_ids = {response_id: response_id for response_id in expected_ids}
        
        start_time = time.time()
        while time.time() - start_time < timeout and len(results) < len(expected_ids):
            rx = self.receive_message(timeout=0.1)
            if rx is None:
                continue
            
            if rx.arbitration_id in expected_response_ids and rx.arbitration_id not in results:
                results[rx.arbitration_id] = rx
        
        for response_id in expected_ids:
            if response_id not in results:
                results[response_id] = None
        
        return results
    
    def __enter__(self):
        if not self.connected:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
    
    def __del__(self):
        self.disconnect()
