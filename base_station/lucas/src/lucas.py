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
import msvcrt
import struct
import sys
import threading
import time

import numpy as np
import pygame
from Sample import Sample
from SimpleSocketServer import SimpleSocketServer
import wifi_mpi
from wifi_mpi import PING, TEXT_STREAM_OUT, Message, process_messages, register_message_handler
import sarah
from sarah import  S_COMPLETE_MOVE, S_FORWARD, S_PING, S_RESET, S_REVERSE, S_STOP, S_TURN_LEFT, S_TURN_RIGHT, SarahState,sarah_inst
from wifi import connect_wifi_windows
from environment_display import SCREEN_SIZE, draw_lidar_pixels, update,screen
from grid import grid_inst
import queue

data_queue = queue.Queue() 



def main():
    parser = argparse.ArgumentParser(description="Connect to open Wi-Fi and start TCP listener (Windows only).")
    parser.add_argument("--ssid", default="Galaxy", help="SSID of the open Wi-Fi network to connect to.") #Vodafone701713
    parser.add_argument("--host", default="0.0.0.0", help="Host/interface to bind (default: localhost).")
    parser.add_argument("--port", type=int, default=9000, help="Port to listen on (default: 8888).")
    parser.add_argument("--no-connect", action="store_true", help="Skip Wi-Fi connection and only run the listener.")
    args = parser.parse_args()
    
    server = SimpleSocketServer()
    server.start()
    wifi_mpi.setup(server)
    
    def message_pump():
        timeNoMessage = 0
        lastMessageTime =  time.time()
        while True:
            
            try:
                
                    
                if not server.client_sock:
                    print("Waiting for client...")
                    timeNoMessage = 0
                    lastMessageTime =  time.time()
                    server.accept()
                    
                if timeNoMessage > 2000:
                    wifi_mpi.send(S_PING)
                    print("no message for sometime...pinging")
              
                    if(sarah_inst.ping_count > 5):
                        print("ping_count > 5, client gone")
                        server.close_client()
                    
                    sarah_inst.ping_count = sarah_inst.ping_count + 1
                    timeNoMessage =0
                while True:
                    # Your message processing
                    if(sarah_inst.next_action is not None):
                        sarah_inst.next_action()  
                        sarah_inst.next_action = None
                    process_messages()
                    timeNoMessage = 0
                    lastMessageTime = time.time()
                    time.sleep(0.01)  # avoid busy-waiting
                
            except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError) as e:
                # Client disconnected
                print("Client disconnected:", e)
                server.close_client()  # clean up
                # loop will automatically go back to accept()
            except TimeoutError as e:
                timeNoMessage = timeNoMessage + time.time()-lastMessageTime
                pass
            except Exception as e:
                # Other unexpected errors
                print("Unexpected error:", e)
                server.close_client()
        
    def pingHandler(m:Message):
        #print(m.data)
        sarah_inst.ping_count = 0
        pass
    def printHandler(m:Message):
        print(m.data)
    register_message_handler(PING,pingHandler)
    register_message_handler(TEXT_STREAM_OUT,printHandler)
    register_message_handler(sarah.R_INIT,sarah.sarah_inst.initializing)
    register_message_handler(sarah.R_EST,sarah.sarah_inst.estimating)
    register_message_handler(sarah.R_GET_CMD,sarah.sarah_inst.get_cmd)
    register_message_handler(sarah.R_MAP_DATA,sarah.sarah_inst.data)
    register_message_handler(sarah.R_POS,sarah.sarah_inst.pos)
    register_message_handler(sarah.R_PREP,sarah.sarah_inst.preparing)
    register_message_handler(sarah.R_READY,sarah.sarah_inst.ready)
    
# Start in a separate thread
    thread = threading.Thread(target=message_pump, daemon=True)
    thread.start()
                      
    connect_wifi_windows(args.ssid)
    
    screen_array = np.full((SCREEN_SIZE, SCREEN_SIZE, 3), [0, 0, 0], dtype=np.uint8)
    pygame.init()
    while True:       
                    
        screen_array[:] =  [0, 0, 0]
        if(sarah.sarah_inst.updated):
            sarah.sarah_inst.updated = False
            print(f"X:{sarah.sarah_inst.cur_x},Y:{sarah.sarah_inst.cur_y},ROT:{sarah.sarah_inst.cur_rot}")
            grid_inst.add(sarah.sarah_inst.current_map,sarah.sarah_inst.cur_x ,sarah.sarah_inst.cur_y ,sarah.sarah_inst.cur_rot,sarah.sarah_inst.map_confidence )
            grid_inst.draw(0,0,screen_array,screen,sarah.sarah_inst.cur_x,sarah.sarah_inst.cur_y  )
            update()  
            pass       
        
        if(sarah.sarah_inst.latest_map_part is not None)
        {
            
            
            
            
            
            
        }
            

        # Draw new points if available
        for event in pygame.event.get():              
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if(  event.type == pygame.KEYDOWN and  event.key == pygame.K_r):
                sarah_inst.cur_x = 0
                sarah_inst.cur_y = 0
                sarah_inst.cur_rot = 0
                sarah_inst.next_action = lambda: wifi_mpi.send(S_RESET)
                grid_inst.setup_grid()
            if(  event.type == pygame.KEYDOWN):
                if(event.key == pygame.K_w):                
                    sarah_inst.next_action = lambda: wifi_mpi.send(S_FORWARD)        
                if(event.key == pygame.K_s):                
                    sarah_inst.next_action = lambda: wifi_mpi.send(S_REVERSE)       
                if(event.key == pygame.K_a):                
                    sarah_inst.next_action = lambda: wifi_mpi.send(S_TURN_LEFT)    
                if(event.key == pygame.K_d):                
                    sarah_inst.next_action = lambda: wifi_mpi.send(S_TURN_RIGHT)   
                if(event.key == pygame.K_SPACE):                
                    sarah_inst.next_action = lambda: wifi_mpi.send(S_STOP)                                
            if(  event.type == pygame.KEYDOWN and  event.key == pygame.K_SPACE and sarah_inst.state == SarahState.MOVING):
                 # blocks until EOF (Ctrl+D on Linux/macOS, Ctrl+Z on Windows)
                
                sarah_inst.next_action = lambda: wifi_mpi.send(S_COMPLETE_MOVE)  
            

        pygame.time.wait(10)  # keep window responsive
        


if __name__ == "__main__":
    main()
