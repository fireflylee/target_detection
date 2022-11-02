from base import Base
import json
import signal

isEnd=False

def quit(signum, frame):
    global isEnd
    isEnd=True

signal.signal(signal.SIGINT, quit)                                
signal.signal(signal.SIGTERM, quit)

if __name__ == '__main__':

    print('running...')
    print()

    import matplotlib.pyplot as plt
    
    path=r'C:\Users\Mario Wang\Desktop\fusion\backup\datas\2022.5.17\\'
    tail="_gazebo_1"
    
    f=open(path+"data_backup"+tail+'.txt', "r")
    content=f.read()
    lines=content.split('\n')

    base = Base()

    plt.figure()
    plt.axis('equal')
    plt.grid(alpha=0.5,linestyle='-.')
    plt.ion()

    for messageStr in lines:
        if messageStr!="":
            print(messageStr)
            print()
            base.update(messageStr)
            plt.clf()
            for fusion in base.fusionTargetPool:
                plt.plot(fusion.fusionPos.x,fusion.fusionPos.y,marker="o", markersize=10)
                plt.text(fusion.fusionPos.x+1,fusion.fusionPos.y+1, str(fusion.id)+('-Lost' if fusion.state=='Lost' else ''))
                print(fusion.id,[(target.uav.id,target.id) for target in fusion.uavTargetPool])
                print('fusionPos:%s; pos:%s'%(fusion.fusionPos,fusion.pos))
                print(fusion.state)
            if len(base.fusionTargetPool)>0:
                print()
            plt.title('t='+str(base.timestamp))
            plt.grid()
            plt.draw()
            plt.axis('equal')
            # plt.xlim(18, 19)
            plt.ylim(-5, 35)
            plt.pause(0.2)
            # plt.show(block=True)
    
    plt.show(block=True)
