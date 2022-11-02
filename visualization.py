import json
import signal
from server import Server_udp
from utilities import pix2pos_2
from configs import *
import matplotlib.pyplot as plt

x_lim=[-60,20]
y_lim=[-50,100]
colors_real=[120/255,120/255,120/255]
colors_fusion=[[80/255,80/255,80/255],[160/255,160/255,160/255],[220/255,220/255,220/255]]
colors=['r','g','b','m','c','k','r','g','b','m','c','k','r','g','b','m','c','k','r','g','b','m','c','k']

isEnd=False

def quit(signum, frame):
    global isEnd
    print('Ctrl-C detected')
    isEnd=True

signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)

fusionJsonString=""
realJsonString=""
uavJsonStrings=["" for _ in range(MAX_UAV_NUM)]

server=Server_udp(port=VISUALIZATION_PORT)

def callback(messageStr):
    if messageStr=="":
        return
    global fusionJsonString,uavJsonStrings,realJsonString
    message=json.loads(messageStr)
    timestamp=message['timestamp']
    if timestamp>=0:
        fusionJsonString=json.dumps(message['fusion'])
        realJsonString=json.dumps(message['realTargets'])
        uavJsonStrings[message['uav']-1]=json.dumps({"uav":message['uav'],"pose":message['pose'],"targets":message['targets']})
    else:
        fusionJsonString=""
        realJsonString=""
        uavJsonStrings=["" for _ in range(MAX_UAV_NUM)]

server.receiveOnThread(callback)

figure=plt.figure('Visualization')
try:
    figure.canvas.manager.window.showMaximized()
except:
    pass
plt.draw()
plt.ion()

while not isEnd:
    plt.clf()

    for i in range(len(uavJsonStrings)):
        uavJsonString=uavJsonStrings[i]
        if uavJsonString!="":
            uav=json.loads(uavJsonStrings[i])
            for uavTarget in uav['targets']:
                plt.plot(uavTarget['x'],uavTarget['y'],marker="x",color=colors[i],markersize=8,linewidth=1.2,label="UAV "+str(uav['uav'])+" Detect "+str(uavTarget['id']))
            pos1,r1=pix2pos_2(uav['pose'],MTXS[i],[0,0],inv_inmtx=INV_MTXS[i])
            pos2,r2=pix2pos_2(uav['pose'],MTXS[i],[0,RESOLUTION[1]-1],inv_inmtx=INV_MTXS[i])
            pos3,r3=pix2pos_2(uav['pose'],MTXS[i],[RESOLUTION[0]-1,RESOLUTION[1]-1],inv_inmtx=INV_MTXS[i])
            pos4,r4=pix2pos_2(uav['pose'],MTXS[i],[RESOLUTION[0]-1,0],inv_inmtx=INV_MTXS[i])
            pos5,r5=pix2pos_2(uav['pose'],MTXS[i],[RESOLUTION[0]/2,RESOLUTION[1]/2],inv_inmtx=INV_MTXS[i])
            if r1 and r2 and r3 and r4:
                xs=[pos1[0],pos2[0],pos3[0],pos4[0],pos1[0]]
                ys=[pos1[1],pos2[1],pos3[1],pos4[1],pos1[1]]
                plt.plot(xs,ys,color=colors[i],linewidth=0.4,label="UAV "+str(uav['uav'])+" FOV")
            #plt.plot(uav['pose'][0],uav['pose'][1],color=colors[i],marker="o",markerfacecolor='none')
            plt.text(uav['pose'][0],uav['pose'][1],'['+str(i+1)+']',verticalalignment="center",horizontalalignment="center",color=colors[i],label="UAV "+str(uav['uav']))
            if r5:
                plt.plot([uav['pose'][0],pos5[0]],[uav['pose'][1],pos5[1]],color=colors[i],linestyle="dashed",linewidth=0.8,label="UAV "+str(uav['uav'])+" Camera optical axis")
                plt.plot(pos5[0],pos5[1],color=colors[i],marker="+",markerfacecolor='none')
    
    if fusionJsonString!="":
        fusions=json.loads(fusionJsonString)['fusions']
        for fusionTarget in fusions:
            if fusionTarget['state']=='Active':
                plt.plot(fusionTarget['x'],fusionTarget['y'],marker="o",color=colors_fusion[0],markerfacecolor='none',markeredgewidth=1.6,markersize=8,linewidth=1,label="Fusion "+str(fusionTarget['id'])+" [Active]")
                plt.text(fusionTarget['x'],fusionTarget['y'],"   ID-"+str(fusionTarget['id']),verticalalignment="center",horizontalalignment="left")

                plt.arrow(fusionTarget['x'],fusionTarget['y'],fusionTarget['vx'],fusionTarget['vy'],head_width = 0.3,head_length = 0.5,fc = 'w',ec = colors_fusion[0])

            elif fusionTarget['state']=='Nearly_Lost':
                plt.plot(fusionTarget['x'],fusionTarget['y'],marker="*",color=colors_fusion[1],markerfacecolor='none',markeredgewidth=1.4,markersize=8,linewidth=1,label="Fusion "+str(fusionTarget['id'])+" [Nearly_lost]")
            elif fusionTarget['state']=='Lost':
                plt.plot(fusionTarget['x'],fusionTarget['y'],marker="v",color=colors_fusion[2],markerfacecolor='none',markeredgewidth=1.6,markersize=8,linewidth=1.2,label="Fusion "+str(fusionTarget['id'])+" [Lost]")

    if realJsonString!="":
        realTargets=json.loads(realJsonString)
        for realTarget in realTargets:
            plt.plot(realTarget['x'],realTarget['y'],marker="s",color=colors_real,markerfacecolor='none',markeredgewidth=1.6,markersize=10,linewidth=0.8,label="Real Target")

    plt.grid(alpha=0.5,linestyle='-.')
    
    # plt.legend(loc='upper right')
    plt.axis('equal')
    plt.xlim(x_lim[0], x_lim[1])
    plt.ylim(y_lim[0], y_lim[1])
    plt.xlabel('x/m')
    plt.ylabel('y/m')
    figure.subplots_adjust(left=0.1, right=0.94, top=0.94, bottom=0.1)
    plt.pause(0.1)

    if not plt.fignum_exists('Visualization'):
        break
