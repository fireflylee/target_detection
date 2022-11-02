import threading
import socket
import json
import math
from time import sleep
from utilities import round_floats
from configs import *

IP_1='192.168.42.112'
IP_2='192.168.42.142'
PORT=2000
BUFSIZE=1024

def caculate(start, end):
    #start[0] = latitude, start[1] = longitude, start[2] = altitude
    C_EARTH = 6378137.0
    pos = [0.00, 0.00, 0.00]
    #通过经纬度计算位置偏差
    deltaLon   = (start[1] - end[1]) / 57.3
    deltaLat   = (start[0] - end[0]) / 57.3

    pos[0] = -1 * deltaLon * C_EARTH * math.cos(start[0]/57.3)
    pos[1] = -1 * deltaLat * C_EARTH
    pos[2] = -1 * (start[2] - end[2])
    return pos

class Client:
    def __init__(self,ip,port,callback):
        self.ip=ip
        self.port=port
        def run():
            while(True):
                self.receive(callback)
        t=threading.Thread(target = run)
        t.daemon = True
        t.start()
    
    def receive(self,callback):
        while True:
            try:
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.connect((self.ip,self.port))
                print('connected')
                while True:
                    data=self.client.recv(BUFSIZE)
                    if len(data)==0:
                        print('close')
                        break
                    string=data.decode('utf-8')

                    if callback!=None:
                        callback(string)
            
            except Exception as err:
                print(err)

            finally:
                self.client.close()
                sleep(0.1)

    def close(self):
        self.client.close()

class CarRTK:
    def __init__(self,ip,port,start):
        self.start=start
        self.E = 0
        self.N = 0
        self.U = 0
        self.YAW = 0
        self.state=0
        self.end=[0,0,0]
        self.ip=ip
        self.msg=""
        self.cl = Client(ip,port,self.callback)

    def callback(self,string):
        self.msg=string
        data_list = string.split(',')
        if len(data_list)>=10:
            lat = int(float(data_list[2]))/100 + (float(data_list[2])/100 - int(float(data_list[2])/100))/0.6
            lon = int(float(data_list[4]))/100 + (float(data_list[4])/100 - int(float(data_list[4])/100))/0.6
            
            self.state=int(data_list[6])
            self.end = [lat, lon, float(data_list[9])]

            car_pos = caculate(self.start, self.end)

            self.E = car_pos[0]
            self.N = car_pos[1]
            self.U = car_pos[2]

if __name__ == "__main__":
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dest_addr = ('192.168.42.255', REAL_PORT)
    
    cars=[CarRTK(IP_1,PORT, [32.03627842866667, 119.36726401516667, 49.883]),
          CarRTK(IP_2,PORT, [32.03857007766666, 119.34752770266665, 7.4714])]

    while True:
        pos_dict = round_floats([{'x':car.E,'y':car.N,'from':car.ip} for car in cars])
        print("--------------------------------")
        for car in cars:
            print(car.ip)
            print("current:"+str(car.end))
            print("x:%.2f y:%.2f state:%d"%(car.E,car.N,car.state))
            print()

        data = json.dumps(pos_dict).encode('utf-8')
        udp_socket.sendto(data, dest_addr)
        sleep(0.1)


