#!/usr/bin/env python
import threading
import pickle
import copy
import argparse
import geometry_msgs.msg
from lmpvc_controller_bridge.comms import Server, Client

class CodeGenWebClient:
    """Provides access to CodeGen over a tcp connection. Mostly useful for prototyping with a 
    remote GPU machine. NOT SECURE, so use e.g. SSH tunneling for access outside LAN.
    """

    def __init__(self, ip='127.0.0.1'):
        
        # Used for most communication
        self.server = Server(5000)
        self.client = Client(ip, 5001)
    
    def def generate_inference(self, prompt, preamble='', policies=''):
        print("\nWeb client: Requesting inference...")
        # Send instruction
        with self.comms_lock:
            msg = pickle.dumps({'command': 'set_speed', 'speed': speed})
            self.client.send(msg)

            (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
        if resp == b'success':
            print("Success!")
            return True
        else:
            print("Failed!")
            return False



