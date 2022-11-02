import json
import signal
from server import Server_udp
from utilities import pos2pix_2
from configs import *
import matplotlib.pyplot as plt
import math
from jsonMessage.JsonReader import JsonReader
from jsonMessage.TrackMessage import TrackMessage

#Set ROW and COLUMN by hand or by default

ROW=3
COLUMN=1

#Use DATA_PORT for raw datas else VISUALIZATION_PORT for datas after fusion

# PORT=DATA_PORT
PORT=VISUALIZATION_PORT

isEnd=False

def quit(signum, frame):
    global isEnd
    print('Ctrl-C detected')
    isEnd=True

signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)

uavJsonStrings=["" for _ in range(MAX_UAV_NUM)]
realJsonString=""

tms=[TrackMessage() for _ in range(MAX_UAV_NUM)]
jrs=[JsonReader(tms[i],40001+i) for i in range(MAX_UAV_NUM)]

server=Server_udp(port=PORT)

def callback(messageStr):
    if messageStr=="":
        return
    global uavJsonStrings,tms,realJsonString
    message=json.loads(messageStr)
    timestamp=message['timestamp']
    if timestamp>=0:
        uavJsonStrings[message['uav']-1]=json.dumps({"pose":message['pose'],"targets":message['targets']})
        if PORT==VISUALIZATION_PORT:
            realJsonString=json.dumps(message['realTargets'])
    else:
        uavJsonStrings=["" for _ in range(MAX_UAV_NUM)]
        realJsonString=""
        for i in range(MAX_UAV_NUM):
            tms[i].isTrack=False

server.receiveOnThread(callback)

if PORT==DATA_PORT:
    server2=Server_udp(port=REAL_PORT)
    def callback2(messageStr):
        if messageStr=="":
            return None

        global realJsonString
        realJsonString=messageStr
        return None
    server2.receiveOnThread(callback2)

figure=plt.figure('CameraTrackerVisualization')
# try:
#     figure.canvas.manager.window.showMaximized()
# except:
#     pass
plt.draw()
plt.ion()

color=[120/255,120/255,120/255]
color_light=[220/255,220/255,220/255]
colors=['r','g','b','m','c','k','r','g','b','m','c','k','r','g','b','m','c','k','r','g','b','m','c','k']
arrowStart=[RESOLUTION[0]/2,RESOLUTION[1]/2]
North=[0,10E6,0]
East=[10E6,0,0]
BORDER1=RESOLUTION[0]/2-100
BORDER2=RESOLUTION[1]/2-100
R=60
EDGE=math.atan2(BORDER2,BORDER1)
XTICKS=[]#[0,RESOLUTION[0]/2,RESOLUTION[0]]
YTICKS=[]#[0,RESOLUTION[1]/2,RESOLUTION[1]]

if not (ROW and COLUMN) or ROW*COLUMN<MAX_UAV_NUM:
    COLUMN=math.ceil(math.sqrt(MAX_UAV_NUM))
    ROW=math.ceil(MAX_UAV_NUM/COLUMN)

def drawAngle(angle,color,text=None,size=8,text_color='b'):
    while angle>math.pi:
        angle-=math.pi*2
    while angle<=-math.pi:
        angle+=math.pi*2

    if angle>=EDGE:
        if angle==math.pi:
            x=-BORDER1
            y=0
        elif angle>=math.pi-EDGE:
            x=-BORDER1
            y=x*math.tan(angle)
        elif angle==math.pi/2:
            y=BORDER2
            x=0
        else:
            y=BORDER2
            x=y/math.tan(angle)
    elif angle<-EDGE:
        if angle<-math.pi+EDGE:
            x=-BORDER1
            y=x*math.tan(angle)
        elif angle==-math.pi/2:
            x=0
            y=-BORDER2
        else:
            y=-BORDER2
            x=y/math.tan(angle)
    elif angle==0:
        x=BORDER1
        y=0
    else:
        x=BORDER1
        y=x*math.tan(angle)

    x+=RESOLUTION[0]/2
    y+=RESOLUTION[1]/2

    x1=R*math.cos(angle+1.57)+x
    y1=R*math.sin(angle+1.57)+y
    x2=R*math.cos(angle)+x
    y2=R*math.sin(angle)+y
    x3=R*math.cos(angle-1.57)+x
    y3=R*math.sin(angle-1.57)+y
    
    if text:
        plt.text(x,y,text,verticalalignment="center",horizontalalignment="center",color=text_color,size=size)
    plt.plot([x1,x2,x3],[y1,y2,y3],linewidth=1.4,color=color)

while not isEnd:
    plt.clf()

    if realJsonString!="":
        realTargets = json.loads(realJsonString)
    else:
        realTargets=[]

    for i in range(MAX_UAV_NUM):
        plt.subplot(ROW,COLUMN,i+1)

        plt.xlim(0,RESOLUTION[0])
        plt.ylim(0,RESOLUTION[1])
        plt.text(0,RESOLUTION[1]+10," UAV "+str(i+1),color=colors[i],verticalalignment="bottom",horizontalalignment="left")

        uavJsonString=uavJsonStrings[i]
        if uavJsonString!="":
            uav=json.loads(uavJsonString)
            pose=uav['pose']

            #Find North
            arrowEnd,isInFOV,isRealPix=pos2pix_2(pose,MTXS[i],North)
            dx=arrowEnd[0]-arrowStart[0]
            dy=arrowStart[1]-arrowEnd[1]
            if not isRealPix:
                dx*=-1
                dy*=-1
            angle=math.atan2(dy,dx)
            drawAngle(angle,color_light,'N',text_color=color)

            #Draw real targets
            for real in realTargets:
                pix,isInFOV,_=pos2pix_2(pose,MTXS[i],[real['x'],real['y'],0])
                if isInFOV:
                    plt.scatter(pix[0],RESOLUTION[1]-pix[1],marker="s",alpha=0.8,s=50,linewidth=1.4,color=color,facecolor='none')
            
            #Draw targets and track by id
            isTrackIdFound=False
            for target in uav['targets']:
                pix,_,_=pos2pix_2(pose,MTXS[i],[target['x'],target['y'],0])
                pix[1]=RESOLUTION[1]-pix[1]
                plt.scatter(pix[0],pix[1],marker="x",color=colors[i],s=50,linewidth=1.4)
                plt.text(pix[0],pix[1],"  "+str(target['id']),color=colors[i],verticalalignment="center",horizontalalignment="left")
                if tms[i].isTrack and tms[i].id==target['id']:
                    isTrackIdFound=True
                    plt.text(pix[0]-10,pix[1]+20,"╔═",verticalalignment="bottom",horizontalalignment="right",color='r')
                    plt.text(pix[0]+10,pix[1]+20,"═╗",verticalalignment="bottom",horizontalalignment="left",color='r')
                    plt.text(pix[0]-10,pix[1]-30,"╚═",verticalalignment="top",horizontalalignment="right",color='r')
                    plt.text(pix[0]+10,pix[1]-30,"═╝",verticalalignment="top",horizontalalignment="left",color='r')

            #Draw track if track by xy
            if tms[i].isTrack and not isTrackIdFound:
                arrowEnd,isInFOV,isRealPix=pos2pix_2(pose,MTXS[i],[tms[i].x,tms[i].y,0])
                if isInFOV:
                    plt.arrow(arrowStart[0],arrowStart[1],arrowEnd[0]-arrowStart[0],arrowStart[1]-arrowEnd[1],head_width=50,width=1,fc = 'w' ,ec = 'r')
                else:
                    dx=arrowEnd[0]-arrowStart[0]
                    dy=arrowStart[1]-arrowEnd[1]
                    if not isRealPix:
                        dx*=-1
                        dy*=-1
                    angle=math.atan2(dy,dx)
                    drawAngle(angle,'r')

        plt.xticks(XTICKS)
        plt.yticks(YTICKS)
        
    # plt.axis('equal')
    figure.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.05)
    plt.pause(0.1)

    if not plt.fignum_exists('CameraTrackerVisualization'):
        break
