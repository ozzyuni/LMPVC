#!/usr/bin/env python3
import sys
import pickle
import codegen_cli as codegen
from comms import Server, Client

class CodeGenWebServer:
    """Provides access to the functionality of controller_client.py (running in ROS1) over a web interface
        THIS IS THE ROS 1 SIDE OF THE BRIDGE
    """

    def __init__(self, ip = '127.0.0.1'):
        """Networking provided by comms.py, ports and server ip (to whcih client connects)
        can be changed here
        """
        # Used for most communication 
        self.server = Server(5006)
        self.client = Client(ip, 5007)
    
    def run(self):
        """Listens to messages formatted as [instruction, data] and implements appropriate behaviour"""

        try:

            while True:
                (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
                if success:
                    msg = pickle.loads(resp)
                
                    print("\nMessage received, generating inference...")
                    resp = codegen.generate_inference(msg[0], msg[1])
                    print("Sending response!")
                    self.client.send(resp.encode('utf-8'))

        except KeyboardInterrupt:
            print("Keyboard interrupt!")

if __name__ == '__main__':
    ip = '127.0.0.1'
    try:
        ip = sys.argv[sys.argv.index('-ip') + 1]
    except IndexError:
        print("No valid -ip argument detected, using default.")

    print("Connecting to", ip)
    bridge = CodeGenWebServer(ip=ip)
    bridge.run()
