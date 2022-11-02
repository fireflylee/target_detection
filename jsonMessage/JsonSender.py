import socket
from .JsonMessageBase import *

class JsonSender():
    def __init__(self,data:JsonMessageBase,ip,port):
        self.data=data
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.dst=(ip,port)

    def sendData(self):
        self.client.sendto((self.data.to_json()).encode('utf-8'), self.dst)
