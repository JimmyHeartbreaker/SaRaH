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



        
