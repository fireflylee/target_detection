import json
import signal
from server import Server_udp
from utilities import pix2pos_2
from configs import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

DRAW_TRAJ=True
TRAJ_LEN=100

# 注释选择绘制场景参数

# 操场
# x_lim=[-60,20]
# y_lim=[-50,100]
# z_lim=[0,30]
# y_pane_color=(0.0, 0.0, 1.0, 0.02)
# x_pane_color=(0.0, 0.0, 1.0, 0.10)
# z_pane_color=(0.0, 1.0, 0.0, 0.05)
# pitch=45
# yaw=-75
# L=-0.0
# R=1.0
# T=1.05
# B=-0.05

# 楼顶
# x_lim=[-25,25]
# y_lim=[-25,25]
# z_lim=[0,20]
# y_pane_color=(0.0, 0.0, 1.0, 0.02)
# x_pane_color=(0.0, 0.0, 1.0, 0.10)
# z_pane_color=(0.0, 1.0, 0.0, 0.05)
# pitch=45
# yaw=-75
# L=0
# R=1
# T=1
# B=0

# gazebo 12
# x_lim=[-200,600]
# y_lim=[-100,600]
# z_lim=[0,200]
# x_pane_color=(0.0, 0.0, 1.0, 0.02)
# y_pane_color=(0.0, 0.0, 1.0, 0.10)
# z_pane_color=(0.0, 1.0, 0.0, 0.05)
# pitch=45
# yaw=-15
# L=-0.05
# R=1.1
# T=1.1
# B=-0.05

# gazebo 24
# x_lim=[-200,900]
# y_lim=[-100,1100]
# z_lim=[0,200]
# x_pane_color=(0.0, 0.0, 1.0, 0.02)
# y_pane_color=(0.0, 0.0, 1.0, 0.10)
# z_pane_color=(0.0, 1.0, 0.0, 0.05)
# pitch=45
# yaw=-15
# L=-0.15
# R=1.2
# T=1.15
# B=-0.15

# gazebo 24 5x5km
# x_lim=[-500,5500]
# y_lim=[-500,5500]
# z_lim=[0,200]
# x_pane_color=(0.0, 0.0, 1.0, 0.02)
# y_pane_color=(0.0, 0.0, 1.0, 0.10)
# z_pane_color=(0.0, 1.0, 0.0, 0.05)
# pitch=45
# yaw=-15
# L=-0.15
# R=1.2
# T=1.15
# B=-0.15

# U3d
x_lim=[-500,8000]
y_lim=[-500,8000]
z_lim=[0,300]
x_pane_color=(0.0, 0.0, 1.0, 0.02)
y_pane_color=(0.0, 0.0, 1.0, 0.10)
z_pane_color=(0.0, 1.0, 0.0, 0.05)
pitch=45
yaw=-15
L=-0.15
R=1.2
T=1.15
B=-0.15

colors_real=[150/255,150/255,150/255]
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

if DRAW_TRAJ:
    paths=[([],[],[]) for _ in range(MAX_UAV_NUM)]
    pathsReal=[([],[],[]) for _ in range(MAX_UGV_NUM)]

server=Server_udp(port=VISUALIZATION_PORT)

isNew=False
def callback(messageStr):
    if messageStr=="":
        return
    global fusionJsonString,uavJsonStrings,realJsonString,paths,pathsReal,isNew
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
        if DRAW_TRAJ:
            paths=[([],[],[]) for _ in range(MAX_UAV_NUM)]
            pathsReal=[([],[],[]) for _ in range(MAX_UGV_NUM)]
    isNew=True

server.receiveOnThread(callback)

figure=plt.figure('Visualization')
figure.subplots_adjust(left=L, right=R, top=T, bottom=B)
try:
    figure.canvas.manager.window.showMaximized()
except:
    pass
plt.draw()
plt.ion()

while not isEnd:
    plt.clf()
    ax = figure.add_subplot(projection='3d')

    for i in range(len(uavJsonStrings)):
        uavJsonString=uavJsonStrings[i]
        if uavJsonString!="":
            uav=json.loads(uavJsonStrings[i])
            #Draw uav ( and trajectory )
            if DRAW_TRAJ:
                paths[uav['uav']-1][0].append(uav['pose'][0])
                paths[uav['uav']-1][1].append(uav['pose'][1])
                paths[uav['uav']-1][2].append(uav['pose'][2])
                if TRAJ_LEN>0 and len(paths[uav['uav']-1][0])>TRAJ_LEN:
                    paths[uav['uav']-1][0].pop(0)
                    paths[uav['uav']-1][1].pop(0)
                    paths[uav['uav']-1][2].pop(0)
                ax.plot(paths[uav['uav']-1][0],paths[uav['uav']-1][1],paths[uav['uav']-1][2],color=colors[i],linewidth=0.3,label="UAV "+str(uav['uav'])+" Traj")
            #Draw uav targets
            for uavTarget in uav['targets']:
                ax.scatter(uavTarget['x'],uavTarget['y'],0,marker="x",color=colors[i],s=50,linewidth=1.2,label="UAV "+str(uav['uav'])+" Detect "+str(uavTarget['id']))
            #Draw FOV
            pos1,r1=pix2pos_2(uav['pose'],MTXS[i],[0,0],inv_inmtx=INV_MTXS[i])
            pos2,r2=pix2pos_2(uav['pose'],MTXS[i],[0,RESOLUTION[1]-1],inv_inmtx=INV_MTXS[i])
            pos3,r3=pix2pos_2(uav['pose'],MTXS[i],[RESOLUTION[0]-1,RESOLUTION[1]-1],inv_inmtx=INV_MTXS[i])
            pos4,r4=pix2pos_2(uav['pose'],MTXS[i],[RESOLUTION[0]-1,0],inv_inmtx=INV_MTXS[i])
            pos5,r5=pix2pos_2(uav['pose'],MTXS[i],[RESOLUTION[0]/2,RESOLUTION[1]/2],inv_inmtx=INV_MTXS[i])
            if r1 and r2 and r3 and r4:
                xs=[pos1[0],pos2[0],pos3[0],pos4[0],pos1[0]]
                ys=[pos1[1],pos2[1],pos3[1],pos4[1],pos1[1]]
                zs=[0,0,0,0,0]
                ax.plot(xs,ys,color=colors[i],linewidth=0.1,label="UAV "+str(uav['uav'])+" FOV")
            # ax.plot(uav['pose'][0],uav['pose'][1],uav['pose'][2],color=colors[i],marker="o",markerfacecolor='none')
            ax.text(uav['pose'][0],uav['pose'][1],uav['pose'][2],'['+str(i+1)+']',verticalalignment="center",horizontalalignment="center",color=colors[i],label="UAV "+str(uav['uav']))
            ax.text(uav['pose'][0],uav['pose'][1],uav['pose'][2],'     '+str(round(uav['pose'][2],2))+'m',size=6,verticalalignment="top",horizontalalignment="left",color=colors[i])
            if r5:
                ax.plot([uav['pose'][0],pos5[0]],[uav['pose'][1],pos5[1]],[uav['pose'][2],0],color=colors[i],linestyle="dotted",linewidth=0.15)#,label="UAV "+str(uav['uav'])+" Camera optical axis")
                ax.plot(pos5[0],pos5[1],0,color=colors[i],marker="+",markerfacecolor='none',alpha=0.4)
            ax.plot([uav['pose'][0],uav['pose'][0]],[uav['pose'][1],uav['pose'][1]],[uav['pose'][2],0],color=colors[i],linestyle="dashed",linewidth=0.2)
            ax.plot(uav['pose'][0],uav['pose'][1],0,color=colors[i],marker="o",markerfacecolor='none',markersize=2,alpha=0.5)
    
    #Draw real targets ( and trajectory )
    if realJsonString!="":
        realTargets=json.loads(realJsonString)
        if DRAW_TRAJ:
            index=0
        for realTarget in realTargets:
            if DRAW_TRAJ:
                pathsReal[index][0].append(realTarget['x'])
                pathsReal[index][1].append(realTarget['y'])
                pathsReal[index][2].append(0)
                if TRAJ_LEN>0 and len(pathsReal[index][0])>TRAJ_LEN:
                    pathsReal[index][0].pop(0)
                    pathsReal[index][1].pop(0)
                    pathsReal[index][2].pop(0)
                ax.plot(pathsReal[index][0],pathsReal[index][1],pathsReal[index][2],color='k',linewidth=0.5,label="UGV "+str(index+1)+" Traj")
                index+=1
            ax.scatter(realTarget['x'],realTarget['y'],0,marker="s",alpha=0.8,s=50,linewidth=1.4,color=colors_real,facecolor='none',label="Real Target")

    #Draw fusion targets
    if fusionJsonString!="":
        fusions=json.loads(fusionJsonString)['fusions']
        for fusionTarget in fusions:
            if fusionTarget['state']=='Active':
                ax.scatter(fusionTarget['x'],fusionTarget['y'],0,marker="o",color=colors_fusion[0],facecolor='none',s=50,linewidth=1.2,label="Fusion "+str(fusionTarget['id'])+" [Active]")
                ax.text(fusionTarget['x'],fusionTarget['y'],0,"   ID-"+str(fusionTarget['id']),verticalalignment="center",horizontalalignment="left")
                ax.quiver(fusionTarget['x'],fusionTarget['y'],0,fusionTarget['vx'],fusionTarget['vy'],0.01, length=4, arrow_length_ratio=0.4, linewidth=0.5, color = colors_fusion[0])

            elif fusionTarget['state']=='Nearly_Lost':
                ax.scatter(fusionTarget['x'],fusionTarget['y'],0,marker="*",color=colors_fusion[1],facecolor='none',s=50,linewidth=1,label="Fusion "+str(fusionTarget['id'])+" [Nearly_lost]")
                ax.text(fusionTarget['x'],fusionTarget['y'],0,"   ID-"+str(fusionTarget['id'])+' [NL]',verticalalignment="center",horizontalalignment="left")

            elif fusionTarget['state']=='Lost':
                ax.scatter(fusionTarget['x'],fusionTarget['y'],0,marker="v",color=colors_fusion[2],facecolor='none',s=50,linewidth=1.2,label="Fusion "+str(fusionTarget['id'])+" [Lost]")

    # ax.grid(False)
    # plt.legend(loc='upper left',bbox_to_anchor=(L,1-(T-1)))
    
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_ylim(y_lim[0], y_lim[1])
    ax.set_zlim(z_lim[0], z_lim[1])
    ax.set_xlabel('x/m')
    ax.set_ylabel('y/m')
    ax.set_zlabel('z/m')
    ax.xaxis._axinfo["grid"].update({"linewidth":0.1})
    ax.yaxis._axinfo["grid"].update({"linewidth":0.1})
    ax.zaxis._axinfo["grid"].update({"linewidth":0.1})
    ax.w_xaxis.set_pane_color(x_pane_color)
    ax.w_yaxis.set_pane_color(y_pane_color)
    ax.w_zaxis.set_pane_color(z_pane_color)
    plt.gca().set_box_aspect((x_lim[1]-x_lim[0], y_lim[1]-y_lim[0], z_lim[1]-z_lim[0]))
    ax.view_init(pitch, yaw)

    isNew=False
    while True:
        plt.pause(0.1)
        if isNew or isEnd or not plt.fignum_exists('Visualization'):
            break

    if not plt.fignum_exists('Visualization'):
        break
