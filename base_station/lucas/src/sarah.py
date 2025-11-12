#!/usr/bin/env python3
"""
wifi_listen_windows.py
Connect to an open Wi-Fi network (no password) and start a TCP listener.

Usage:
    python wifi_listen_windows.py --ssid "MyOpenSSID" --port 8888
    python wifi_listen_windows.py --no-connect --port 8888
"""


from enum import Enum

S_ACK = 0x01
S_NACK = 0x02
S_INIT = 0x01
S_READY = 0x02
S_PREP = 0x03
S_MOVE = 0x04
S_EST = 0x05
S_GET_CMD = 0x06
S_DATA = 0x07
class SarahState(Enum):     
    BOOT=0
    INITIALIZING=1
    PREPARING=2
    READY=3
    MOVING=4
    SCANNING=5
    ESTIMATING=6
    
class Sarah:
    state = SarahState.BOOT
    cur_x = 0
    cur_y = 0
    cur_rot = 0
    
    d_x = 0
    d_y = 0
    d_rot =0.78


sarah_inst = Sarah()
