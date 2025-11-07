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
import subprocess
import threading
import time

import numpy as np
import pygame
from environment_display import SCREEN_SIZE, draw_lidar_pixels, update
from message_handler import handle_received_data, data_queue
from socketserver import ThreadingTCPServer, StreamRequestHandler


def connect_wifi_windows(ssid, timeout=20):
    """Attempt to connect on Windows using netsh (best-effort)."""
    print(f"[windows] trying to connect to SSID '{ssid}' ...")
    try:
        # netsh requires a saved profile for some networks; if it exists, this works directly
        cmd = f'netsh wlan connect ssid="{ssid}" name="{ssid}"'
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, shell=True)
        if res.returncode == 0:
            print("[windows] netsh reports success:")
            print(res.stdout.strip())
            return True
        else:
            print("[windows] netsh failed:")
            print(res.stderr.strip() or res.stdout.strip())
            print("[hint] You may need to create a WLAN profile or connect manually once via Windows UI.")
            return False
    except Exception as e:
        print("[windows] exception while connecting:", e)
        return False



    
class EchoHandler(StreamRequestHandler):
    """Handles each client connection: prints and echoes received data."""
    def handle(self):
        peer = self.client_address
        print(f"[connection] {peer} connected")
        try:
            print(f"bufsize {self.rbufsize}")
                
            if self.rfile.readable():
                data1 = self.rfile.read(8*1440)
                if data1:
                    handle_received_data(data1, peer)
                data2 = self.rfile.read(8*1440)
                if data2:
                    handle_received_data(data2, peer)
                
                data3 = self.rfile.read(8*1440)
                

                if data3:
                    handle_received_data(data3, peer)

                # Optionally send a response back
                response = b"ACK\n"
                self.wfile.write(response)
               
        except Exception as e:
            self.rfile.flush()
            print(f"[connection] error from {peer}: {e}")
        finally:
            print(f"[connection] {peer} disconnected")



def run_server(host="0.0.0.0", port=8888):
    """Start a TCP server that prints incoming data."""
    print(f"[server] listening on {host}:{port} (Ctrl-C to stop)")
    ThreadingTCPServer.allow_reuse_address = True
    try:
        with ThreadingTCPServer((host, port), EchoHandler) as server:
            server.serve_forever()
    except KeyboardInterrupt:
        print("\n[server] stopped by user")
    except Exception as e:
        print(f"[server] failed to start: {e}")
        



def main():
    parser = argparse.ArgumentParser(description="Connect to open Wi-Fi and start TCP listener (Windows only).")
    parser.add_argument("--ssid", default="Galaxy", help="SSID of the open Wi-Fi network to connect to.")
    parser.add_argument("--host", default="0.0.0.0", help="Host/interface to bind (default: localhost).")
    parser.add_argument("--port", type=int, default=9000, help="Port to listen on (default: 8888).")
    parser.add_argument("--no-connect", action="store_true", help="Skip Wi-Fi connection and only run the listener.")
    args = parser.parse_args()

    if args.ssid and not args.no_connect:
        print("[main] attempting Wi-Fi connection...")
        ok = connect_wifi_windows(args.ssid)
        if not ok:
            print("[main] Wi-Fi connection failed or not confirmed.")
        time.sleep(2)  # let the interface settle
   
        
    server_thread = threading.Thread(target=run_server, args=(args.host, args.port), daemon=True)
    server_thread.start()
    screen_array = np.full((SCREEN_SIZE, SCREEN_SIZE, 3), [0, 0, 0], dtype=np.uint8)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Draw new points if available
        if not data_queue.empty():
            screen_array[:] =  [0, 0, 0]
            old = data_queue.get_nowait()   
            while data_queue.empty():  
                pass
            new = data_queue.get_nowait()  
            while data_queue.empty():
                pass
            transformed = data_queue.get_nowait()
            draw_lidar_pixels(old,screen_array,[0, 255, 0] )
            draw_lidar_pixels(new,screen_array,[0, 0, 255] )
            draw_lidar_pixels(transformed,screen_array,[255, 0, 0] )
            update()

        pygame.time.wait(10)  # keep window responsive


if __name__ == "__main__":
    main()
