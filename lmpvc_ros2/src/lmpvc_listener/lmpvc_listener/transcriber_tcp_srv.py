#!/usr/bin/env python3
import sys
import pickle
import lmpvc_codegen.codegen import CodeGen
from pathlib import Path
from lmpvc_codegen.comms import Server, Client

class TranscriberWebServer:
    """Provides access to CodeGen over a tcp connection. Mostly useful for prototyping with a 
    remote GPU machine. NOT SECURE, so use e.g. SSH tunneling for access outside LAN.
    """

    def __init__(self, node=None, ip = '127.0.0.1'):
        """Networking provided by comms.py, ports and server ip (to whcih client connects)
        can be changed here
        """
        # Used for most communication
        self.node = node
        self.server = Server(5003)
        self.client = Client(ip, 5004)
        self.asr = lmpvc_listener.whisper.SpeechRecognition()
    
    def log_info(msg):
        if self.node is not None:
            self.node.get_logger().info(msg)
        else:
            print("INFO: " + msg)

    def log_error(msg):
        if self.node is not None:
            self.node.get_logger().error(msg)
        else:
            print("ERROR: " + msg)

    def run(self):
        """Listens to messages formatted as [instruction, data] and implements appropriate behaviour"""

        try:

            while True:
                (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
                if success:
                    self.log_info("Received request to transcribe")
                    msg = pickle.loads(resp)
                
                    filename = Path(__file__).with_name('temp.wav')

                    with open(filename, 'wb') as audiofile:
                        audiofile.write(b''.join(msg))
                    
                    transcript = self.asr.generate_transcript(str(filename))
                    self.log_info("Finished transcribing: " + transcript)
                    filename.unlink()

                    self.client.send(transcript.encode('utf-8'))

        except KeyboardInterrupt:
            self.log_error("Keyboard interrupt")

def main():
    rclpy.init()

    node = Node('transcriber_web_server')

    ip = '127.0.0.1'
    try:
        ip = sys.argv[sys.argv.index('-ip') + 1]
    except IndexError:
        print("No valid -ip argument detected, using default.")

    rclpy.spin(node)

    bridge = TrascriberWebServer(node=node, ip=ip)
    bridge.log_info("Connecting to " + ip)
    bridge.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()