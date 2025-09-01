#!/usr/bin/env python3
import sys
import pickle
import lmpvc_codegen.codegen import CodeGen
from lmpvc_codegen.comms import Server, Client

class CodeGenWebServer:
    """Provides access to CodeGen over a tcp connection. Mostly useful for prototyping with a 
    remote GPU machine. NOT SECURE, so use e.g. SSH tunneling for access outside LAN.
    """

    def __init__(self, ip = '127.0.0.1'):
        """Networking provided by comms.py, ports and server ip (to whcih client connects)
        can be changed here
        """
        # Used for most communication 
        self.server = Server(5001)
        self.client = Client(ip, 5002)
        self.codegen = CodeGen()
    
    def run(self):
        """Listens to messages formatted as [instruction, data] and implements appropriate behaviour"""

        try:

            while True:
                (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
                if success:
                    msg = pickle.loads(resp)
                
                    print("\nMessage received, generating inference...")
                    result = self.codegen.generate_inference(msg[0], msg[1], msg[2], log=True)
                    print("Sending response!")
                    self.client.send(resp.encode('utf-8'))

        except KeyboardInterrupt:
            print("Keyboard interrupt!")

def main():
    ip = '127.0.0.1'
    try:
        ip = sys.argv[sys.argv.index('-ip') + 1]
    except IndexError:
        print("No valid -ip argument detected, using default.")

    print("Connecting to", ip)
    bridge = CodeGenWebServer(ip=ip)
    bridge.run()

if __name__ == '__main__':
    main()
