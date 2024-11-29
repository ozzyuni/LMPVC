#!/usr/bin/env python
import select
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM

class Server:
    """ A simple TCP server application for receiving message blocks of a specified length. 
        Includes verification that the entire message was received.
    """

    def __init__(self, port: int, block_size=512):
        """Set connection parameters and bind a port for listening
            Args:
                port | number of the TCP port the server will listen on
                block_size=512 | size of the message block to be received (in bytes), make sure this is long enough
        """
        self.hostname = gethostbyname('0.0.0.0')
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.bind((self.hostname, port))
        self.start_tag = '<message>'.encode('utf-8')
        self.end_tag = '</message>'.encode('utf-8')
        self.block_size = block_size
    
    def receive(self, timeout=False, timeout_in_seconds=3.0):
        """Receive data from the bound port
            Args:
                timeout | if set, the server will stop listening after a specified time
                timeout_in_seconds=3.0 | sets the number of seconds the server will wait before timeout
            Returns:
                data | the content of the message (bytes)
                success | True if verification was successful, False otherwise
        """
        ready = True
        success = False
        data = b'None'

        if timeout:
            ready = select.select([self.socket], [], [], timeout_in_seconds)[0]

        if ready:
            (data, addr) = self.socket.recvfrom(self.block_size)

            if data[:len(self.start_tag)] == self.start_tag and data[-len(self.end_tag):] == self.end_tag:
                success = True
                data = data[len(self.start_tag):-len(self.end_tag)]
                padding_size = int.from_bytes(data[:2], byteorder='little')
                data = data[2:-padding_size]

        return data, success

class Client:
    """ A simple TCP client application for sending message blocks of a specified length """
    def __init__(self, ip: str, port: int, block_size=512):
        """Set connection parameters
            Args:
                ip | ip address of the server the client should connect to
                port | number of the TCP port the server will listen on
                block_size=512 | size of the message block to be sent (in bytes), make sure this is long enough
        """
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.connect((ip, port))
        self.ip = ip
        self.port = port
        self.start_tag = '<message>'.encode('utf-8')
        self.end_tag = '</message>'.encode('utf-8')
        self.block_size = block_size
         
    def send(self, data: bytes):
        """Send data to server at ip:port
            Args:
                data | contents of the message (bytes)
                NOTICE: to successfully send data, make sure that len(data) < block_size
        """
        padding_size = self.block_size - (len(self.start_tag) + len(data) + len(self.end_tag)) - 2
        padding_data = padding_size.to_bytes(2, byteorder='little')
        padding = bytes(padding_size)
        msg = self.start_tag + padding_data + data + padding + self.end_tag
        self.socket.sendto(msg,(self.ip,self.port))