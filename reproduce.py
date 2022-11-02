import socket
from time import sleep
from configs import *
from utilities import getProgressString,tailMethod4timedelta
import signal
import os
import json

isEnd=False

def quit(signum, frame):
    global isEnd
    print('Ctrl-C detected')
    isEnd=True

signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)

path=r'E:\xfd_program\fusion_v7\fusion_v7\backup\datas\2022.10.30\\'
tail="_16.29.42"
interval=0.2/MAX_UAV_NUM

SEND_UDP_DIRECTLY=False     #直接发送原始UDP
SEND_AIRPLANE_STATE=True    #发送airplane状态包

lines=[]
lines2=[]
lines3=None

f=open(path+"data_backup"+tail+'.txt', "r")
content=f.read()
lines=content.split('\n')
f.close()

f=open(path+"real_backup"+tail+'.txt', "r")
content=f.read()
lines2=content.split('\n')
f.close()

if os.path.exists(path+"track_backup"+tail+'.txt'):
    
    from jsonMessage.JsonSender import JsonSender
    from jsonMessage.TrackMessage import TrackMessage

    f=open(path+"track_backup"+tail+'.txt', "r")
    content=f.read()
    lines3=content.split('\n')
    f.close()

i=len(lines)
if len(lines2)<i:
    i=len(lines2)
if lines3:
    if len(lines3)<i:
        i=len(lines3)

lines=lines[0:i]
lines2=lines2[0:i]
if lines3:
    lines3=lines3[0:i]
    
    tms=[TrackMessage() for _ in range(MAX_UAV_NUM)]
    jss=[JsonSender(tms[i],'127.255.255.255',40001+i) for i in range(MAX_UAV_NUM)]

if SEND_AIRPLANE_STATE:
    import struct
    import math

    airplane_state_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(MAX_UAV_NUM)]
    airplane_state_pose=[None for _ in range(MAX_UAV_NUM)]
    for s in airplane_state_sockets:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    airplane_state_addrs = [('127.255.255.255', 33201+i) for i in range(MAX_UAV_NUM)]

def sendTrack(i):
    if lines3 and lines3[i]!="":
        try:
            uav,isTrack,x,y,id=json.loads(lines3[i])
            tms[uav].isTrack=isTrack
            tms[uav].x=x
            tms[uav].y=y
            tms[uav].id=id
            jss[uav].sendData()
        except:
            tracks=json.loads(lines3[i])
            for j in range(MAX_UAV_NUM):
                if isinstance(tracks[j],list):
                    tms[j].isTrack=True
                    tms[j].x=tracks[j][0]
                    tms[j].y=tracks[j][1]
                    tms[j].id=tracks[j][2]
                    jss[j].sendData()
                else:
                    if tms[j].isTrack!=False:
                        tms[j].isTrack=False
                        tms[j].x=0
                        tms[j].y=0
                        tms[j].id=-1
                        jss[j].sendData()

def sendAirplaneState(i):
    message=json.loads(lines[i])
    if 'pose' in message:
        index=message['uav']-1
        x,y,h,_,_,_=message['pose']
        if airplane_state_pose[index]!=None:
            dy=y-airplane_state_pose[index][1]
            dx=x-airplane_state_pose[index][0]
            if not (dx==0 and dy==0):
                yaw=math.atan2(dy,dx)/math.pi*180+270
                pitch=math.atan2(h-airplane_state_pose[index][2],math.sqrt(dx**2+dy**2))/math.pi*180
            else:
                yaw=airplane_state_pose[index][3]
                pitch=airplane_state_pose[index][4]
        else:
            yaw=0
            pitch=0
        airplane_state_pose[index]=(x,y,h,yaw,pitch,0)
        data=struct.pack('ddddddB',y/111000.0,x/111000.0,h,yaw,pitch,0,0 if h==0 else ( 2 if h>-30 else 3 ))
        airplane_state_sockets[index].sendto(data,airplane_state_addrs[index])

if SEND_UDP_DIRECTLY:

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dest_addr = ('127.255.255.255', DATA_PORT)
    dest_addr2 = ('127.255.255.255', REAL_PORT)

    times=0
    while not isEnd:
        times+=1
        print("times:"+str(times))
        if SEND_AIRPLANE_STATE:
            airplane_state_pose=[None for _ in range(MAX_UAV_NUM)]
        for i in range(len(lines)):
            text=lines2[i]#+(1024-len(lines2[i].encode('utf-8')))*' '
            udp_socket.sendto(text.encode('utf-8'), dest_addr2)
            text=lines[i]#+(1024-len(lines[i].encode('utf-8')))*' '
            udp_socket.sendto(text.encode('utf-8'), dest_addr)

            sendTrack(i)
            if SEND_AIRPLANE_STATE:
                sendAirplaneState(i)

            sleep(interval)
            if isEnd:
                break

    udp_socket.close()

else:

    from base import Base
    from utilities import round_floats
    import os

    realTargets=[]
    outputs=[]
    base=Base()

    print("[Done] %d lines found." % len(lines))
    print("       Loading results...")

    total=len(lines)*interval
    for i in range(len(lines)):

        if isEnd:
            exit()

        now=(i+1)*interval

        print(getProgressString(now,total),end='    \r',flush=True)

        if lines2[i]!="":
            realTargets=json.loads(lines2[i])

        if lines[i]=="":
            continue

        message=json.loads(lines[i])
        timestamp=message['timestamp']
        if timestamp>=0:
            base.update(lines[i])
            message['fusion']={'fusions':[{'id':fusion.id,'x':fusion.fusionPos.x,'y':fusion.fusionPos.y,'z':fusion.fusionPos.z,'vx':fusion.fusionSpd[0],'vy':fusion.fusionSpd[1],
                               'from':[(target.uav.id,target.id) for target in fusion.uavTargetPool] if fusion.state=='Active' else [],'state':fusion.state} 
                               for fusion in base.fusionTargetPool]}
            # message['fusion']={'fusions':[{'id':fusion.id,'x':fusion.pos.x,'y':fusion.pos.y,'z':fusion.pos.z,'vx':fusion.fusionSpd[0],'vy':fusion.fusionSpd[1],
            #                    'from':[(target.uav.id,target.id) for target in fusion.uavTargetPool] if fusion.state=='Active' else [],'state':fusion.state} 
            #                    for fusion in base.fusionTargetPool]}
            message['realTargets']=realTargets
            outputs.append(json.dumps(round_floats(message)))
        else:
            outputs.append(lines[i])

    print()
    print("[Done] Results loaded.")
    print()
    print("       Right →:  10s forward    10s+")
    print("        Left ←:  10s backword   10s-")
    print("          Up ↑:  speed up        >+ ")
    print("        Down ↓:  speed down      >- ")
    print("             P:  Pause           || ")
    print("             R:  Restart         |< ")
    print("             Q:  Quit            /\\ ")
    print()
    print("       Date:"+path.split(os.sep)[-3]+' tail:'+tail)

    class KeyReader:

        def __init__(self):
            import threading
            self.keys=[]
            tr = threading.Thread(target=self.__run, args=())
            tr.start()

        def getKeys(self):
            result=[]
            while len(self.keys)>0:
                result.append(self.keys.pop(0))
            return result

        def __find_getch(self):
            try:
                import termios 
            except ImportError:
                # Non-POSIX. Return msvcrt's (Windows') getch.
                import msvcrt        
                return msvcrt.getch    # POSIX system. Create and return a getch that manipulates the tty.
            import sys, tty
            self.__fd = sys.stdin.fileno()
            self.__old_settings = termios.tcgetattr(self.__fd)
            tty.setraw(self.__fd)
            def _getch():
                ch = sys.stdin.read(1)
                return ch.encode('ascii')
            return _getch

        def __recover(self):
            try:
                import termios
                termios.tcsetattr(self.__fd, termios.TCSADRAIN, self.__old_settings)
            except:
                pass

        def __run(self):

            global isEnd
            
            getChar=self.__find_getch()
            char=b'\x00'
            key="other"
            while not isEnd:
                char=getChar()
                if isEnd:
                    break
                if char==b'\x03':
                    isEnd=True
                    self.__recover()
                elif char==b'P' or char==b'p':
                    key='P'
                elif char==b'R' or char==b'r':
                    key='R'
                elif char==b'Q' or char==b'q':
                    key='Q'
                    isEnd=True
                    self.__recover()
                elif char==b'\x00' or char==b'\xe0':
                    char = getChar()
                    if char==b'H':      #up 
                        key="up"
                    elif char==b'P':    #down
                        key="down"
                    elif char==b'K':    #left
                        key="left"
                    elif char==b'M':    #right
                        key="right"
                    else:
                        key="other"
                else:
                    if char==b'\x1B':
                        char = getChar()
                        if char==b'[':
                            char = getChar()
                            if char==b'A':      #up 
                                key="up"
                            elif char==b'B':    #down
                                key="down"
                            elif char==b'D':    #left
                                key="left"
                            elif char==b'C':    #right
                                key="right"
                            else:
                                key='other'
                        else:
                            key="other"
                    else:
                        key="other"
                self.keys.append(key)
            self.__recover()

    
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dst=('127.255.255.255', VISUALIZATION_PORT)

    locator="(+)"
    front="#"
    
    total=len(outputs)*interval
    kr=KeyReader()
    isRestart=False
    isPause=False
    speed=10
    speedCoef=1.0
    while not isEnd:
        isRestart=False
        i=0
        if SEND_AIRPLANE_STATE:
            airplane_state_pose=[None for _ in range(MAX_UAV_NUM)]
        while i<len(outputs):

            while not isEnd:
                for key in kr.getKeys():
                    if key=='R':
                        isRestart=True
                        break
                    elif key=='P':
                        isPause=not isPause
                    elif key=='Q':
                        isEnd=True
                    elif key=='left':
                        i=int(i-10/interval)
                        if i<0:
                            i=0
                    elif key=='right':
                        i=int(i+10/interval)
                        if i>=len(outputs):
                            i=len(outputs)-1
                    elif key=='up':
                        if speed<19:
                            speed+=1
                        speedCoef=1.0/(speed-9) if speed>=10 else 10.0/speed
                    elif key=='down':
                        if speed>1:
                            speed-=1
                        speedCoef=1.0/(speed-9) if speed>=10 else 10.0/speed
                now=(i+1)*interval
                print(getProgressString(now,total,head=" Pause " if isPause else ("%6s "%("x"+str(speed-9)+'.0') if speed>=10 else "%6s "%("x0."+str(speed))),
                      front=front,locator=locator,tailMethod=tailMethod4timedelta),
                      end='    \r',flush=True)
                if not isPause or isRestart or isEnd:
                    break
                elif isPause:
                    sleep(0.1)

            if isRestart or isEnd:
                break

            client.sendto(outputs[i].encode('utf-8'), dst)

            sendTrack(i)
            if SEND_AIRPLANE_STATE:
                sendAirplaneState(i)
                            
            sleep(interval*speedCoef)
            i+=1

    print()
    print()
