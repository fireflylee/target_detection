from sklearn.gaussian_process.kernels import RBF
from sklearn.gaussian_process import GaussianProcessRegressor
import numpy as np
import matplotlib.pyplot as plt
import json

DATA_START=100
DATA_LEN=-1
MAX_DELAY=2000
UAV_ID=1
TARGET_ID=1

AVR_LEN=6
INTERVAL=2
SIGMA=2000
SIGMA2=2000
DT=5
PATH=r'C:\Users\Mario Wang\Desktop\fusion\backup\datas\2022.5.19\\'
TAIL="_gazebo_3"

def GPR(src_x:list,src_y:list,axisX:list,sigma):
    train_X = np.array(src_x).reshape(-1, 1)
    train_y = [[y] for y in src_y]

    test_X = np.array(axisX).reshape(-1, 1)

    sca_l = sigma  #sklearn内置以标准差为参数
    ga = 1/(2*sca_l*sca_l) #python内置的rbf核函数为gamma参数

    ker = RBF(length_scale=sca_l, length_scale_bounds='fixed')
    gpr = GaussianProcessRegressor(kernel=ker, n_restarts_optimizer=2,normalize_y=False)
    gpr.fit(train_X, train_y)
    mu, _ = gpr.predict(test_X, return_cov=True)
    test_y = mu.ravel()

    return test_y

def fromDataGetV():
    f=open(PATH+r'data_backup'+TAIL+'.txt', "r")
    content=f.read()
    lines=content.split('\n')
    f.close()
    f=open(PATH+r'real_backup'+TAIL+'.txt', "r")
    content=f.read()
    lines2=content.split('\n')
    f.close()

    if DATA_LEN>=0 and DATA_LEN<len(lines)-1:
        lines=lines[DATA_START:DATA_LEN]
        lines2=lines2[DATA_START:DATA_LEN]
    else:
        lines=lines[DATA_START:]
        lines2=lines2[DATA_START:]

    datas=[]

    for i in range(len(lines)-1):
        data=json.loads(lines[i])
        if data['timestamp']>0 and data['uav']==UAV_ID and len(data['targets'])>TARGET_ID-1:
            real=json.loads(lines2[i])
            datas.append({'timestamp':data['timestamp'],'x':data['targets'][0]['x'],'y':data['targets'][0]['y'],'rx':real[TARGET_ID-1]['x'],'ry':real[TARGET_ID-1]['y']})

    axisOffset=datas[0]['timestamp']

    axisX=[]
    Vx=[]
    Vy=[]
    RealVx=[]
    RealVy=[]
    for i in range(len(datas)-AVR_LEN-INTERVAL):
        vx=0
        vy=0
        rvx=0
        rvy=0
        axisX.append(datas[i]['timestamp']-axisOffset)
        for j in range(AVR_LEN):
            vx+=(datas[i+j+INTERVAL]['x']-datas[i+j]['x'])/(datas[i+j+INTERVAL]['timestamp']-datas[i+j]['timestamp'])
            vy+=(datas[i+j+INTERVAL]['y']-datas[i+j]['y'])/(datas[i+j+INTERVAL]['timestamp']-datas[i+j]['timestamp'])
            rvx+=(datas[i+j+INTERVAL]['rx']-datas[i+j]['rx'])/(datas[i+j+INTERVAL]['timestamp']-datas[i+j]['timestamp'])
            rvy+=(datas[i+j+INTERVAL]['ry']-datas[i+j]['ry'])/(datas[i+j+INTERVAL]['timestamp']-datas[i+j]['timestamp'])
        Vx.append(vx/AVR_LEN*1000)
        Vy.append(vy/AVR_LEN*1000)
        RealVx.append(rvx/AVR_LEN*1000)
        RealVy.append(rvy/AVR_LEN*1000)

    return axisX,Vx,Vy,RealVx,RealVy

def fromDataGetS():
    f=open(PATH+r'data_backup'+TAIL+'.txt', "r")
    content=f.read()
    lines=content.split('\n')
    f.close()
    f=open(PATH+r'real_backup'+TAIL+'.txt', "r")
    content=f.read()
    lines2=content.split('\n')
    f.close()

    if DATA_LEN>=0 and DATA_LEN<len(lines)-1:
        lines=lines[DATA_START:DATA_LEN]
        lines2=lines2[DATA_START:DATA_LEN]
    else:
        lines=lines[DATA_START:]
        lines2=lines2[DATA_START:]

    datas=[]

    for i in range(len(lines)-1):
        data=json.loads(lines[i])
        if data['timestamp']>0 and data['uav']==UAV_ID and len(data['targets'])>TARGET_ID-1:
            real=json.loads(lines2[i])
            datas.append({'timestamp':data['timestamp'],'x':data['targets'][0]['x'],'y':data['targets'][0]['y'],'rx':real[TARGET_ID-1]['x'],'ry':real[TARGET_ID-1]['y']})

    axisOffset=datas[0]['timestamp']

    axisX=[]
    X=[]
    Y=[]
    RealX=[]
    RealY=[]
    for i in range(len(datas)-AVR_LEN-1):
        x=0
        y=0
        rx=0
        ry=0
        axisX.append(datas[i]['timestamp']-axisOffset)
        for j in range(AVR_LEN):
            x+=datas[i+j]['x']
            y+=datas[i+j]['y']
            rx+=datas[i+j]['rx']
            ry+=datas[i+j]['ry']
        X.append(x/AVR_LEN)
        Y.append(y/AVR_LEN)
        RealX.append(rx/AVR_LEN)
        RealY.append(ry/AVR_LEN)

    return axisX,X,Y,RealX,RealY

def getDelay(xx,yy,yy_real):
    min=-1
    min_index=-1
    for i in range(int(MAX_DELAY/DT)):
        sum=0
        for j in range(len(xx)-i):
            sum+=(yy[j+i]-yy_real[j])**2
        if min<0 or min>sum:
            min=sum
            min_index=i
    return min_index*DT

plt.figure()

axisX,Vx,Vy,RealVx,RealVy=fromDataGetV()

XX=[i*DT for i in range(0,int(axisX[-1]/DT))]
YY_Vx=GPR(axisX,Vx,XX,sigma=SIGMA)
YY_Vy=GPR(axisX,Vy,XX,sigma=SIGMA)
YY_Vrx=GPR(axisX,RealVx,XX,sigma=SIGMA)
YY_Vry=GPR(axisX,RealVy,XX,sigma=SIGMA)

axisX=[i/1000 for i in axisX]
XX=[i/1000 for i in XX]

plt.subplot(411)

plt.plot(XX, YY_Vx, label="predict vx")
plt.plot(XX, YY_Vrx, label="predict vrx")
plt.scatter(axisX, Vx, label="vx", marker="*")
plt.scatter(axisX, RealVx, label="vrx", marker=".")
plt.legend()

plt.subplot(412)

plt.plot(XX, YY_Vy, label="predict vy")
plt.plot(XX, YY_Vry, label="predict vry")
plt.scatter(axisX, Vy, label="vy", marker="*")
plt.scatter(axisX, RealVy, label="vry", marker=".")
plt.legend()

axisX2,X,Y,RealX,RealY=fromDataGetS()

XX2=[i*DT for i in range(0,int(axisX2[-1]/DT))]
YY_X=GPR(axisX2,X,XX2,sigma=SIGMA2)
YY_Y=GPR(axisX2,Y,XX2,sigma=SIGMA2)
YY_RX=GPR(axisX2,RealX,XX2,sigma=SIGMA2)
YY_RY=GPR(axisX2,RealY,XX2,sigma=SIGMA2)

axisX2=[i/1000 for i in axisX2]
XX2=[i/1000 for i in XX2]

plt.subplot(413)

plt.plot(XX2, YY_X, label="predict x")
plt.plot(XX2, YY_RX, label="predict rx")
plt.scatter(axisX2, X, label="x", marker="*")
plt.scatter(axisX2, RealX, label="rx", marker=".")
plt.legend()

plt.subplot(414)

plt.plot(XX2, YY_Y, label="predict y")
plt.plot(XX2, YY_RY, label="predict ry")
plt.scatter(axisX2, Y, label="y", marker="*")
plt.scatter(axisX2, RealY, label="ry", marker=".")

plt.legend()

delay_vx=getDelay(XX,YY_Vx,YY_Vrx)
print("Delay from Vx: "+str(delay_vx)+" ms")
delay_vy=getDelay(XX,YY_Vy,YY_Vry)
print("Delay from Vy: "+str(delay_vy)+" ms")
delay_x=getDelay(XX2,YY_X,YY_RX)
print("Delay from X: "+str(delay_x)+" ms")
delay_y=getDelay(XX2,YY_Y,YY_RY)
print("Delay from Y: "+str(delay_y)+" ms")
print("Average delay: "+str(int((delay_vx+delay_vy+delay_x+delay_y)/4))+" ms")

plt.show()