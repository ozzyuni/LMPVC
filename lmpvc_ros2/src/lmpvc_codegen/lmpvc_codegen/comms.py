#!/usr/bin/env python
import select
import socket
class Server:
    """ A simple TCP server application for receiving message blocks of a specified length. 
        Includes verification that the entire message was received.
    """

    def __init__(self, port: int, block_size=512, socket_type='tcp'):
        """Set connection parameters and bind a port for listening
            Args:
                port | number of the TCP port the server will listen on
                block_size=512 | size of the message block to be received (in bytes), make sure this is long enough
        """
        self.hostname = socket.gethostbyname('0.0.0.0')
        
        if socket_type == 'tcp':
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind((self.hostname, port))
            self.guest_socket = None
            self.socket.listen(5)
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.hostname, port))
        
        self.socket_type = socket_type
        
        self.start_tag = '<message>'.encode('utf-8')
        self.end_tag = '</message>'.encode('utf-8')
        self.block_size = block_size
    
    def receive_udp(self, timeout=False, timeout_in_seconds=3.0):
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
    
    def receive_tcp(self, msg_len, timeout=False, timeout_in_seconds=3.0):
        success = True
        data = b'None'

        print("MSG_LEN: ", msg_len)

        chunks = []
        bytes_recd = 0
        while bytes_recd < msg_len:
            chunk = self.guest_socket.recv(min(msg_len - bytes_recd, 2048))
            if chunk == b'':
                print("Socket connection broken")
                success = False
                break
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        
        if success:
            data = b''.join(chunks)

        print("MSG: ", data)

        return data, success

    def respond(self, data: bytes):
        success = False

        if self.send_tcp(len(data).to_bytes(4096, byteorder='little'), 4096):
            success = self.send_tcp(data, len(data))

        self.guest_socket.close()

        return success


    def send_tcp(self, msg, msg_len):
        success = True

        totalsent = 0
        while totalsent < msg_len:
            sent = self.guest_socket.send(msg[totalsent:])
            if sent == 0:
                print("socket connection broken")
                success = False
                break
            totalsent = totalsent + sent

        return success


    def receive(self, timeout=False, timeout_in_seconds=3.0):
        """Receive data from the bound port
            Args:
                timeout | if set, the server will stop listening after a specified time
                timeout_in_seconds=3.0 | sets the number of seconds the server will wait before timeout
            Returns:
                data | the content of the message (bytes)
                success | True if verification was successful, False otherwise
        """
        success = True
        data = b'None'

        if self.socket_type == 'tcp':
            if timeout:
                self.socket.settimeout(timeout_in_seconds)

            try:
                (self.guest_socket, addr) = self.socket.accept()
            except socket.timeout:
                print("Socket timeout")
                success = False
                return data, success

            msg_len_data, success = self.receive_tcp(4096, timeout, timeout_in_seconds)
            
            if success:
                data, success = self.receive_tcp(int.from_bytes(msg_len_data, 'little'), timeout, timeout_in_seconds)
        
        else:
            data, success = self.receive_udp(timeout, timeout_in_seconds)

        return data, success

class Client:
    """ A simple TCP client application for sending message blocks of a specified length """
    def __init__(self, ip: str, port: int, block_size=512, timeout=False, timeout_in_seconds=3.0, socket_type='tcp'):
        """Set connection parameters
            Args:
                ip | ip address of the server the client should connect to
                port | number of the TCP port the server will listen on
                block_size=512 | size of the message block to be sent (in bytes), make sure this is long enough
        """
        if socket_type == 'tcp':
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.connect((ip, port))

        self.socket_type = socket_type
        self.ip = ip
        self.port = port
        self.start_tag = '<message>'.encode('utf-8')
        self.end_tag = '</message>'.encode('utf-8')
        self.block_size = block_size
    
    def send(self, data: bytes, timeout=False, timeout_in_seconds=3.0):

        success = None
        resp = None

        if self.socket_type == 'tcp':
            success = False
            resp = b'None'

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            if timeout:
                self.socket.settimeout(timeout_in_seconds)

            self.socket.connect((self.ip, self.port))

            if self.send_tcp(len(data).to_bytes(4096, byteorder='little'), 4096):
                if(self.send_tcp(data, len(data))):
                    msg_len_data, success = self.receive_tcp(4096, timeout, timeout_in_seconds)

                    if success:
                        resp, success = self.receive_tcp(int.from_bytes(msg_len_data, 'little'), timeout, timeout_in_seconds)

            self.socket.close()
        else:
            self.send_udp(data)

        return resp, success

    def send_udp(self, data: bytes):
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
    
    def send_tcp(self, msg, msg_len):
        success = True

        print("MSG_LEN: ", msg_len)
        print("MSG: ", msg)

        totalsent = 0
        while totalsent < msg_len:
            sent = self.socket.send(msg[totalsent:])
            if sent == 0:
                print("socket connection broken")
                success = False
                break
            totalsent = totalsent + sent

        return success

    def receive_tcp(self, msg_len, timeout=False, timeout_in_seconds=3.0):
        success = True
        data = b'None'

        chunks = []
        bytes_recd = 0
        while bytes_recd < msg_len:
            chunk = self.socket.recv(min(msg_len - bytes_recd, 2048))
            if chunk == b'':
                print("Socket connection broken")
                success = False
                break
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        
        if success:
            data = b''.join(chunks)

        return data, success