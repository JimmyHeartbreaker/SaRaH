import ctypes
import queue

import pygame

from Sample import Sample
data_queue = queue.Queue() 

def handle_received_data(data: bytes, peer):
    """
    Process received data from a client connection.
    
    Parameters:
        data (bytes): Raw bytes received from the client.
        peer (tuple): (ip, port) of the connected client.
    """
    try:
        samples = [
           Sample.from_bytes(data, offset=i * ctypes.sizeof(Sample))
            for i in range(1440)
        ]
        
        data_queue.put(samples)
      
    except Exception as e:
        print(f"[handler error] failed to process data from {peer}: {e}")