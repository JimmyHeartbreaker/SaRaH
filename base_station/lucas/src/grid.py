#!/usr/bin/env python3
"""
wifi_listen_windows.py
Connect to an open Wi-Fi network (no password) and start a TCP listener.

Usage:
    python wifi_listen_windows.py --ssid "MyOpenSSID" --port 8888
    python wifi_listen_windows.py --no-connect --port 8888
"""

import argparse
import csv
import ctypes
from io import BufferedIOBase
import struct

import numpy as np
import pygame
from Sample import Sample
import sarah
from sarah import  SarahState,sarah_inst
from environment_display import SCREEN_SIZE, draw_lidar_pixels, update

RES = 25 #mm
SIZE = 10000 #mm
WIDTH = int(SIZE/RES)
class GridCell():
    size = SIZE
    occupancy =0
    
class Grid():    
    
    def setup_grid(self):
        
        self.data = np.empty((WIDTH, WIDTH), dtype=object)
        
    def print_samples_csv(self,samples, filename=None):
        """
        Prints or saves all Sample objects in self.data to CSV.
        Each line will be: x,y
        """
        rows = []
        for sample in samples:
                
                if sample is not None:
                    rows.append([sample.x, sample.y])

        if filename:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x','y'])  # header
                writer.writerows(rows)
            print(f"Saved {len(rows)} samples to {filename}")
        else:
            print('x,y')
            for row in rows:
                print(f"{row[0]},{row[1]}")
        
    def reinforce(self,samples:list[Sample],x,y,rot):
        for sample in samples:
            rx = sample.x * np.cos(rot) - sample.y * np.sin(rot)
            ry = sample.x * np.sin(rot) + sample.y * np.cos(rot)
            
            tx = rx + x
            ty = ry + y
            
            xi = int(np.round(tx / RES) + WIDTH /2)
            yi = int(np.round(ty / RES) + WIDTH /2)
            if self.data[xi,yi] is not None:
                if(self.data[xi,yi].occupancy  > 0):
                    self.data[xi,yi].occupancy = self.data[xi,yi].occupancy * 2
            
        
        
    def add(self,samples:list[Sample],x,y,rot,confidence):
        for sample in samples:
            rx = sample.x * np.cos(rot) - sample.y * np.sin(rot)
            ry = sample.x * np.sin(rot) + sample.y * np.cos(rot)
            
            tx = rx + x
            ty = ry + y
            
            xi = int(np.round(tx / RES) + WIDTH /2)
            yi = int(np.round(ty / RES) + WIDTH /2)
            if self.data[xi,yi] is None:
                self.data[xi,yi] = GridCell()
            self.data[xi,yi].occupancy = self.data[xi,yi].occupancy + confidence*2
            
        def ocUpdate(cell):
            if cell is not None:
                if cell.occupancy <= confidence*2:#looks like a single hit, probably noise, rub it out
                    cell.occupancy = 0
                #cell.occupancy = max(0,cell.occupancy / 2)
        vecFn = np.vectorize(ocUpdate)
        vecFn(self.data)
            
        

    def draw(self,center_x,center_y,screen_array,screen:pygame.Surface,lidar_x,lidar_y):
        screen_array.fill(0)  # black background
      
        cam_world_x = SIZE/2
        cam_world_y = SIZE/2
        cam_width = 1000
        viewport_x_left =  cam_width/2
        viewport_x_right = cam_width
        viewport_y_top = cam_width/2
        viewport_y_bottom = cam_width
        zoom = 0.5
    # Compute half width in cells to center the view
        
        def draw_thing(world_x, world_y,color):
            viewport_x = int(world_x * zoom  + viewport_x_left)
            viewport_y = int(cam_width - (world_y * zoom + viewport_y_top))
            if(viewport_x < 0 or viewport_y < 0 or viewport_x > cam_width or viewport_y > cam_width):
                return

            # Draw a square of size 'scale'
            viewport_x_end = int(min(viewport_x + RES* zoom, screen_array.shape[0]))
            viewport_y_end = int(min(viewport_y + RES* zoom, screen_array.shape[1]))

            screen_array[viewport_x:viewport_x_end, viewport_y:viewport_y_end] =color
        def draw_cell(cell_x,cell_y,color):
            world_x = (cell_x * RES) - SIZE/2  
            world_y = (cell_y * RES) - SIZE/2 
            
            draw_thing(world_x,world_y,color)
            
        draw_thing(lidar_x-RES/2,lidar_y-RES/2, [255, 0, 0])    
        
        for xi in range(WIDTH):
            for yi in range(WIDTH):
                cell = self.data[xi, yi]
                if cell is not None and cell.occupancy > 0:
                    gray = min(255, cell.occupancy)
                    draw_cell(xi,yi, [gray, gray, gray])
                    # Convert grid indices to screen coordinates
                    
        
        # # Blit once
        pygame.surfarray.blit_array(screen, screen_array)   
    
grid_inst = Grid()
grid_inst.setup_grid()