import socket
from .JsonMessageBase import *

BUFSIZE=2048

class JsonReader():
    def __init__(self,data:JsonMessageBase,port):
        self.data=data
        self.server=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0',port))
        self.receiveOnThread()
    
    def receive(self):
        try:
            data=self.server.recv(BUFSIZE)
            string=data.decode('utf-8').replace('\0','')
            self.data.readData(string)
            
        except Exception as err:
            print(err)

    def receiveOnThread(self):
        import threading
        def run():
            while(True):
                self.receive()
        t=threading.Thread(target = run)
        t.daemon = True
        t.start()