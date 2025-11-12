#!/usr/bin/env python3
"""
wifi_listen_windows.py
Connect to an open Wi-Fi network (no password) and start a TCP listener.

Usage:
    python wifi_listen_windows.py --ssid "MyOpenSSID" --port 8888
    python wifi_listen_windows.py --no-connect --port 8888
"""

import argparse
import ctypes
from io import BufferedIOBase
import struct

import numpy as np
import pygame
from Sample import Sample
import sarah
from sarah import  SarahState,sarah_inst
from server import start_server, data_queue
from environment_display import SCREEN_SIZE, draw_lidar_pixels, update,screen
from grid import grid_inst

def convertToSamples(data):
    return [
        Sample.from_bytes(data, offset=(i) * ctypes.sizeof(Sample))
        for i in range(1440)
    ]
def main():
    parser = argparse.ArgumentParser(description="Connect to open Wi-Fi and start TCP listener (Windows only).")
    parser.add_argument("--ssid", default="Vodafone701713", help="SSID of the open Wi-Fi network to connect to.") #
    parser.add_argument("--host", default="0.0.0.0", help="Host/interface to bind (default: localhost).")
    parser.add_argument("--port", type=int, default=9000, help="Port to listen on (default: 8888).")
    parser.add_argument("--no-connect", action="store_true", help="Skip Wi-Fi connection and only run the listener.")
    args = parser.parse_args()
    def handler(rfile:BufferedIOBase):
        response = bytes([sarah.S_ACK,sarah.S_ACK,sarah.S_ACK,sarah.S_ACK])
        data = rfile.read(4)
        match sarah_inst.state:                       
            case SarahState.BOOT:
                expected = sarah.S_INIT
                nextState = SarahState.INITIALIZING   
            case SarahState.INITIALIZING:
                expected = sarah.S_PREP
                nextState = SarahState.PREPARING    
            case SarahState.PREPARING:
                expected = sarah.S_READY   
                nextState = SarahState.READY                     
            case SarahState.READY:
                expected = sarah.S_GET_CMD
                response = struct.pack('<ifff', 0x01010101,-sarah.sarah_inst.d_x, -sarah.sarah_inst.d_y, -sarah.sarah_inst.d_rot)
                nextState = SarahState.ESTIMATING
            case SarahState.MOVING:
                expected = sarah.S_EST
                nextState = SarahState.ESTIMATING
            case SarahState.ESTIMATING:                
                if data[0] == sarah.S_DATA:
                    data_queue.put(rfile.read(1440*8))
                    data = rfile.read(4)
                    data_queue.put(rfile.read(1440*8))
                    data = rfile.read(4)
                    data_queue.put(rfile.read(1440*8))                    
                    data = rfile.read(16)      
                    posrot = struct.unpack('<ffff',data)
                    data_queue.put(posrot)        
                    data = rfile.read(4)      
                              
                    expected = sarah.S_READY
                    nextState = SarahState.READY
        
       
        if(data[0] ==  sarah.S_INIT):
            sarah_inst.state = SarahState.INITIALIZING   
        elif(data[0] == expected):            
            sarah_inst.state = nextState
        else:
            response = bytes([sarah.S_NACK,sarah.S_NACK,sarah.S_NACK,sarah.S_NACK])
        return response
                            
    start_server(handler,args.ssid,args.host,args.port)
    
    screen_array = np.full((SCREEN_SIZE, SCREEN_SIZE, 3), [0, 0, 0], dtype=np.uint8)
    while True:
           
                  
        screen_array[:] =  [0, 0, 0]
       
        if( not data_queue.empty()):
            old = data_queue.get(True)
            oldSamples = convertToSamples(old)
           # draw_lidar_pixels(,screen_array,[0, 255, 0] )       
            new = data_queue.get(True)   
        #    draw_lidar_pixels(convertToSamples(new),screen_array,[0, 0, 255] )        
            transformed = data_queue.get(True)          
            transformedSamples = convertToSamples(transformed)        
          #  draw_lidar_pixels(transformedSamples,screen_array,[255, 0, 0] )
                    
            posyaw = data_queue.get(True)  
            grid_inst.reinforce(oldSamples,sarah.sarah_inst.cur_x ,sarah.sarah_inst.cur_y ,sarah.sarah_inst.cur_rot)
            grid_inst.add(transformedSamples,sarah.sarah_inst.cur_x ,sarah.sarah_inst.cur_y ,sarah.sarah_inst.cur_rot,posyaw[3] )
            sarah.sarah_inst.cur_x = sarah.sarah_inst.cur_x - posyaw[0]
            sarah.sarah_inst.cur_y = sarah.sarah_inst.cur_y - posyaw[1]
            sarah.sarah_inst.cur_rot = sarah.sarah_inst.cur_rot - posyaw[2]
            grid_inst.draw(0,0,screen_array,screen,sarah.sarah_inst.cur_x,sarah.sarah_inst.cur_y  )
            update()    
            
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Draw new points if available
        

        pygame.time.wait(10)  # keep window responsive


if __name__ == "__main__":
    main()
