import socket
import struct

class Server_udp:
    def __init__(self,port):
        self.server=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        socket.SO_REUSEPORT=15
        #self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.server.settimeout(10)
        self.server.bind(('0.0.0.0',port))
    
    def receive(self,callback,dst=None):
        try:
            data=self.server.recv(1024)
            callback(data)
        except Exception as err:
            print(err)

    def receiveOnThread(self,callback):
        import threading
        def run():
            while(True):
                self.receive(callback)
        t=threading.Thread(target = run)
        t.daemon = True
        t.start()

class PoseReader:
    x=0
    y=0
    height=0
    yaw=0
    pitch=0
    roll=0
    def __init__(self,id):
        server=Server_udp(33200+id)
        def callback(data):
            if(len(data)>=48):
                self.x=struct.unpack('d', data[8:16])[0]*111000
                self.y=struct.unpack('d', data[0:8])[0]*111000
                self.height=struct.unpack('d', data[16:24])[0]
                self.yaw=-struct.unpack('d', data[24:32])[0]
                self.pitch=struct.unpack('d', data[32:40])[0]
                self.roll=struct.unpack('d', data[40:48])[0]
        server.receiveOnThread(callback)

class GimbalReader:
    yaw=0
    pitch=0
    roll=0
    def __init__(self,id):
        server=Server_udp(33600+id)
        def callback(data):
            if(len(data)>=24):
                self.yaw=struct.unpack('d', data[0:8])[0]
                self.pitch=struct.unpack('d', data[8:16])[0]
                self.roll=struct.unpack('d', data[16:24])[0]
        server.receiveOnThread(callback)

class CarReader:
    x=0
    y=0
    yaw=0
    def __init__(self,id):
        server=Server_udp(33300+id)
        def callback(data):
            if(len(data)>=24):
                self.x=struct.unpack('d', data[0:8])[0]
                self.y=struct.unpack('d', data[8:16])[0]
                self.yaw=struct.unpack('d', data[16:24])[0]
        server.receiveOnThread(callback)

if __name__=="__main__":
    from time import sleep,time
    import json

    PRs=[PoseReader(i+1) for i in range(12)]
    GRs=[GimbalReader(i+1) for i in range(12)]
    CRs=[CarReader(i+1) for i in range(4)]

    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dst=('192.168.42.255', 6000)
    dst2=('127.255.255.255', 5000)

    while True:
        message=[{'x':car.x,'y':car.y} for car in CRs]
        print(message)
        client.sendto(json.dumps(message).encode('utf-8'), dst)

        # for i in range(12):
        #     message={'timestamp':int(round(time() * 1000)),'uav':int(i+1),'pose':[PRs[i].x,PRs[i].y,PRs[i].height,GRs[i].yaw+PRs[i].yaw,GRs[i].pitch,GRs[i].roll],'targets':[]}
        #     print(message)
        #     client.sendto(json.dumps(message).encode('utf-8'), dst2)

        sleep(0.2)

