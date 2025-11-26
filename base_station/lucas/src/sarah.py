#!/usr/bin/env python3
"""
wifi_listen_windows.py
Connect to an open Wi-Fi network (no password) and start a TCP listener.

Usage:
    python wifi_listen_windows.py --ssid "MyOpenSSID" --port 8888
    python wifi_listen_windows.py --no-connect --port 8888
"""


import ctypes
from enum import Enum
import struct
from typing import Callable, Optional

from Sample import Sample
import wifi_mpi
from wifi_mpi import Message

S_ACK = 0x01
S_NACK = 0x02
S_MOVE = 0x03
S_COMPLETE_MOVE = 0x04
S_RESET = 0x05
S_PING = 0x06

S_STOP = 0x07
S_TURN_LEFT = 0x08
S_TURN_RIGHT = 0x09
S_REVERSE = 0x10
S_FORWARD = 0x11

R_INIT = 0x01
R_READY = 0x02
R_PREP = 0x03
R_MOVE = 0x04
R_EST = 0x05
R_GET_CMD = 0x06
R_MAP_DATA = 0x07
R_POS = 0x08

class SarahState(Enum):     
    BOOT=0
    INITIALIZING=1
    PREPARING=2
    READY=3
    SCANNING=4
    ESTIMATING=5
    MOVING=6
    
def convertToSamples(data):
    return [
        Sample.from_bytes(data, offset=(i) * ctypes.sizeof(Sample))
        for i in range(1440)
    ]
    
class Sarah:
    state = SarahState.BOOT
    prev_x = 0
    cur_x = 0
    cur_y = 0
    cur_rot = 0
    updated = False
    current_map = list[Sample]
    latest_map_part:list[Sample] = None
    map_confidence= 0
    d_x = 95
    d_y = 0
    d_rot =0
    move_completed = False
    next_action:Optional[Callable] = None
    ping_count =0
    
    def initializing(self,m:Message):
        self.state = SarahState.INITIALIZING
        wifi_mpi.send(S_ACK)  
        print("init")
        
    def ready(self,m:Message):
        self.state = SarahState.READY
        wifi_mpi.send(S_ACK) 
        print("ready")
        
    def preparing(self,m:Message):
        self.state = SarahState.PREPARING
        wifi_mpi.send(S_ACK)     
        print("prep")     
        
    def estimating(self,m:Message):
        self.state = SarahState.ESTIMATING
        wifi_mpi.send(S_ACK)   
        print("ack")         
        
    def get_cmd(self,m:Message):
        response = struct.pack('<fff', self.d_x, self.d_y, self.d_rot)
        self.state = SarahState.MOVING
        wifi_mpi.send_bytes(S_MOVE,response)     
        print("press any key when move completed")        
        
    def data(self,m:Message):   
        self.latest_map_part = convertToSamples(m.data)   
        pass
    def pos(self,m:Message):   
        posrotconfidence = struct.unpack('<ffff',m.data)    
        self.cur_x = self.cur_x - posrotconfidence[0]
        self.cur_y = self.cur_y - posrotconfidence[1]
        self.cur_rot = self.cur_rot - posrotconfidence[2]
        self.map_confidence = posrotconfidence[3]
        self.updated = True
        
            
sarah_inst = Sarah()
