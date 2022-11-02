from base import Base
from configs import *
from utilities import round_floats
import signal
import json

isEnd=False

def quit(signum, frame):
    global isEnd
    print('Ctrl-C detected')
    isEnd=True

signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)

if __name__ == '__main__':

    from server import Server_udp
    import socket
    import sys,os
    import time
    from time import sleep

    print('-------------------------------------')
    print('            >>> START <<<            ')
    print('-------------------------------------')
    print()

    base = Base()
    server=Server_udp(port=DATA_PORT)
    server_realTarget=Server_udp(port=REAL_PORT)

    #可视化
    client_visualization = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_visualization.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dst_visualization=('127.255.255.255', VISUALIZATION_PORT)
    
    #决策
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dst=('192.168.42.255', 33333)

    if SAVE_DATA:
        path = sys.path[0]+os.sep+"backup"+os.sep+"datas"+os.sep+time.strftime('%Y.%m.%d',time.localtime(time.time()))
        if not os.path.exists(path):
            os.makedirs(path)

        tail=time.strftime('_%H.%M.%S',time.localtime(time.time()))

        pathOfData=path+os.sep+'data_backup'+tail+'.txt'
        pathOfReal=path+os.sep+'real_backup'+tail+'.txt'
        pathOfTrack=path+os.sep+'track_backup'+tail+'.txt'
        f1=open(pathOfData,'w')
        f2=open(pathOfReal,'w')
        f3=open(pathOfTrack,'w')

        f1.write(json.dumps({'timestamp':-1})+'\n')
        f2.write(json.dumps([])+'\n')
        f3.write('\n')

        from jsonMessage.JsonReader import JsonReader
        from jsonMessage.TrackMessage import TrackMessage

        tms=[TrackMessage() for _ in range(MAX_UAV_NUM)]
        jrs=[JsonReader(tms[i],40001+i) for i in range(MAX_UAV_NUM)]

    jsonBuffer=[]
    realTargets=[]

    def callback(messageStr):
        if messageStr=="":
            return None

        jsonBuffer.append(messageStr)
    
    def callback2(messageStr):
        if messageStr=="":
            return None

        global realTargets
        message=json.loads(messageStr)
        # for i in range(len(message)-1,-1,-1):
        #     if message[i]['from']=='192.168.42.142':
        #         message.pop(i)
        # print(message)
        realTargets=message
        return None

    server.receiveOnThread(callback)
    server_realTarget.receiveOnThread(callback2)

    while not isEnd:

        while len(jsonBuffer)>0:
            messageStr=jsonBuffer.pop(0)

            message=json.loads(messageStr)
            timestamp=message['timestamp']
            if timestamp>=0:

                if SAVE_DATA:
                    f1.write(messageStr+'\n')
                    f2.write(str(realTargets).replace('\'','\"')+'\n')

                    uav=message['uav']-1
                    f3.write(str(json.dumps(round_floats([uav,tms[uav].isTrack,tms[uav].x,tms[uav].y,tms[uav].id])))+'\n')

                # print(message)
                # print()
                
                base.update(messageStr)
                
                for fusion in base.fusionTargetPool:
                    print(fusion.id,[(target.uav.id,target.id) for target in fusion.uavTargetPool])
                    print(fusion.pos)
                    print(fusion.timestamp)
                    print(fusion.state)
                if len(base.fusionTargetPool)>0:
                    print('-------------------------------------')

                message['fusion']={'fusions':[{'id':fusion.id,'x':fusion.fusionPos.x,'y':fusion.fusionPos.y,'z':fusion.fusionPos.z,'vx':fusion.fusionSpd[0],'vy':fusion.fusionSpd[1],
                                              'from':[(target.uav.id,target.id) for target in fusion.uavTargetPool] if fusion.state=='Active' else [],'state':fusion.state} 
                                              for fusion in base.fusionTargetPool]}
                message['realTargets']=realTargets
                client_visualization.sendto(json.dumps(round_floats(message)).encode('utf-8'), dst_visualization)

                # Not-Lost Fusions
                message['fusion']={'fusions':[{'id':fusion.id,'x':fusion.fusionPos.x,'y':fusion.fusionPos.y,'z':fusion.fusionPos.z,'vx':fusion.fusionSpd[0],'vy':fusion.fusionSpd[1],
                                               'from':[(target.uav.id,target.id) for target in fusion.uavTargetPool] if fusion.state=='Active' else [],'state':fusion.state} 
                                               for fusion in base.fusionTargetPool if fusion.state!='Lost']}
                client.sendto(json.dumps(round_floats(message['fusion'])).encode('utf-8'), dst)

                #if base.isUpdated:
                #    base.isUpdated=False

            else:
                print('-------------------------------------')
                print('            >>> RESET <<<            ')
                print('-------------------------------------')
                base=Base()

        sleep(0.001)

    if SAVE_DATA:
        f1.close()
        f2.close()
        f3.close()
