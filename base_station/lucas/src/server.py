#!/usr/bin/env python3
"""
wifi_listen_windows.py
Connect to an open Wi-Fi network (no password) and start a TCP listener.

Usage:
    python wifi_listen_windows.py --ssid "MyOpenSSID" --port 8888
    python wifi_listen_windows.py --no-connect --port 8888
"""

import subprocess
import threading
import time
import queue
import sarah
import struct

from sarah import Sarah, SarahState,sarah_inst

import numpy as np
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


data_queue = queue.Queue() 

    
class MessageHandlerProxy(StreamRequestHandler):
    def __init__(self, *args, handler=None, **kwargs):
        self.handler = handler  # store injected dependency
        super().__init__(*args, **kwargs)
        
    """Handles each client connection: prints and echoes received data."""
    def handle(self):
        peer = self.client_address
        print(f"[connection] {peer} connected")
        while True:
            try:
                print(f"bufsize {self.rbufsize}")
                    
                if self.rfile.readable():
                    response = self.handler(self.rfile); 
                    self.wfile.write(response)
                        
            except Exception as e:
                self.rfile.flush()
                print(f"[connection] error from {peer}: {e}")
            



def run_server(handler,host="0.0.0.0", port=8888):
    """Start a TCP server that prints incoming data."""
    print(f"[server] listening on {host}:{port} (Ctrl-C to stop)")
    ThreadingTCPServer.allow_reuse_address = True
    def handler_factory(*args, **kwargs):
        return MessageHandlerProxy(*args, handler=handler, **kwargs)
    try:
        with ThreadingTCPServer((host, port), handler_factory) as server:
            server.serve_forever()
    except KeyboardInterrupt:
        print("\n[server] stopped by user")
    except Exception as e:
        print(f"[server] failed to start: {e}")
        


def start_server(handler,ssid, host, port):
    
    attempts =5
    if ssid:
        print("[main] attempting Wi-Fi connection...")
        ok = connect_wifi_windows(ssid)
        while not ok and attempts > 0:
            print("[main] Wi-Fi connection failed or not confirmed. retrying")
            time.sleep(2)  # let the interface settle
            attempts = attempts - 1
        if not ok:
            print("[main] Wi-Fi connection failed or not confirmed. giving up")
            return False
            
   
        
    server_thread = threading.Thread(target=run_server, args=(handler,host, port), daemon=True)
    server_thread.start()
    
