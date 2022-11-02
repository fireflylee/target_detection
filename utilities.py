import numpy as np
import math
from datetime import timedelta
from configs import RESOLUTION

def round_floats(o):
    if isinstance(o, float): return round(o, 3)
    if isinstance(o, dict): return {k: round_floats(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)): return [round_floats(x) for x in o]
    return o

def getProgressString(now:float,total:float=None,head="\r       ",locator=">",front=">",back="-",width=40,tailMethod=None,isTotalWidth=False):
    length=width-len(locator)
    if tailMethod:
        tail=tailMethod(now,total)
        if isTotalWidth:
            length-=len(tail)+1
        if total:
            frontLen=int(now/total*length)
        else:
            frontLen=0
        return head+frontLen*front+locator+(length-frontLen)*back+" "+tail
    else:
        if total:
            tail=str(int(now/total*100))+" %"
            if isTotalWidth:
                length-=len(tail)+1
            frontLen=int(now/total*length)
        else:
            tail=str(int(now))
            if isTotalWidth:
                length-=len(tail)+1
            frontLen=0
        return head+frontLen*front+locator+(length-frontLen)*back+" "+tail

def tailMethod4timedelta(now:float,total:float=None):
    nowString=str(timedelta(seconds=int(now)))
    if total:
        totalString=str(timedelta(seconds=int(total)))
        return nowString+" / "+totalString
    else:
        return nowString+" / inf"

class Position:
    x=0
    y=0
    z=0

    def __init__(self,x=0,y=0,z=0):
        self.x=x
        self.y=y
        self.z=z

    @staticmethod 
    def zero():
        return Position()

    def clone(self):
        return Position(self.x,self.y,self.z)

    def getDistanceTo(self,other):
        return math.sqrt((other.x-self.x)**2+(other.y-self.y)**2+(other.z-self.z)**2)

    def getSquareDistanceTo(self,other):
        return (other.x-self.x)**2+(other.y-self.y)**2+(other.z-self.z)**2

    def tolist(self):
        return [self.x,self.y,self.z]

    def __add__(self,other):  
        if isinstance(other, Position):
            return Position(self.x+other.x,self.y+other.y,self.z+other.z)
        else:
            return self
    __radd__ = __add__

    def __sub__(self,other):
        return Position(self.x-other.x,self.y-other.y,self.z-other.z)

    def __mul__(self,other):
        return Position(self.x*other,self.y*other,self.z*other)
    __rmul__=__mul__

    def __abs__(self):
        return self.x**2+self.y**2+self.z**2
    
    def __truediv__(self,other):
        return Position(self.x/other,self.y/other,self.z/other)

    def __str__(self):
        return ('x=%f,y=%f,z=%f'%(self.x,self.y,self.z))

class KalmanFilter:
    def __init__(self,pos,P=np.eye(6)*100,R=np.eye(3)*0.1):
        self.X=np.mat([pos.x,pos.y,pos.z,0,0,0]).T
        self.P=P
        self.R=R
        self.H=np.mat([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]])

    def update(self,pos,dt):
        z=np.mat(pos.tolist()).T
        F=np.mat([[1,0,0,dt,0,0],[0,1,0,0,dt,0],[0,0,1,0,0,dt],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        Q=np.mat([[dt**4/4,0,0,dt**3/2,0,0],[0,dt**4/4,0,0,dt**3/2,0],[0,0,dt**4/4,0,0,dt**3/2],[dt**3/2,0,0,dt**2,0,0],[0,dt**3/2,0,0,dt**2,0],[0,0,dt**3/2,0,0,dt**2]])

        X1=F*self.X
        P1=F*self.P*F.T+Q
        K=P1*self.H.T*np.linalg.pinv(self.H*P1*self.H.T+self.R)
        self.X=X1+K*(z-self.H*X1)
        self.P=P1-K*self.H*P1

def getMaxMatchNum(nx,ny,edge):
    cx, cy={key:-1 for key in nx},{key:-1 for key in ny}
    visited={key:0 for key in ny}
    res=0
    def path(u):
        for v in ny:
            if v in edge[u] and (not visited[v]):
                visited[v]=1
                if cy[v]==-1:
                    cx[u] = v
                    cy[v] = u
                    return 1
                else:
                    if path(cy[v]):
                        cx[u] = v
                        cy[v] = u
                        return 1
        return 0
    for i in nx:
        if cx[i]==-1:
            for key in ny:
                visited[key]=0
            res+=path(i)
    return res

def getAllMaxMatch(nx,ny,edge):
    maxMatchNum=getMaxMatchNum(nx,ny,edge)
    result=[]
    
    visited={key:0 for key in ny}
    matches=[]

    def findMatchFrom(uIndex,isFirst=False):
        u=nx[uIndex]
        if uIndex<len(nx)-1:
            nextUIndex=uIndex+1
        else:
            nextUIndex=-1
        for v in edge[u]:
            if not visited[v]:
                visited[v]=1
                matches.append((u,v))
                if len(matches)==maxMatchNum:
                    result.append(matches.copy())
                elif nextUIndex>0:
                    findMatchFrom(nextUIndex)
                visited[v]=0
                matches.pop()
        if not isFirst and nextUIndex>0:
            findMatchFrom(nextUIndex)

    for uIndex in range(0,len(nx)-maxMatchNum+1):
        findMatchFrom(uIndex,True)

    return (maxMatchNum,result)

def simplifyFromEntire(nx,ny,edge):   # byref nx, byref ny, edge
    popped_nx=[]
    popped_ny=[]
    popped_edge={}
    if len(nx)>0:
        next_us=[]
        next_vs=[]
        u=nx.pop(0)
        popped_nx.append(u)
        next_us.append(u)
        popped_edge[u]=edge[u]
        while len(next_us)>0:
            for u in next_us:
                for v in edge[u]:
                    if v in ny and not v in popped_ny:
                        next_vs.append(v)
                        popped_ny.append(v)
                        ny.remove(v)
            next_us=[]
            for u in nx:
                if not u in popped_edge and any(value in edge[u] for value in next_vs):
                    popped_nx.append(u)
                    next_us.append(u)
                    popped_edge[u]=edge[u]
                    nx.remove(u)
    return popped_nx,popped_ny,popped_edge

def pix2pos(pose:list,FOV,pix:list):

    resolution=np.array(RESOLUTION)-1

    x,y,height,yaw,pitch,roll=pose

    yaw=yaw*0.017453292519943
    pitch=pitch*0.017453292519943
    roll=roll*0.017453292519943
    _FOV=np.array(FOV)*0.017453292519943

    oriImgPixPos=np.array([-resolution[0]/2+pix[0],resolution[1]/2-pix[1]]).T

    d=height/np.sin(-pitch)
    imgRealMeasurement=np.array([d*np.tan(_FOV[0]/2)*2,d*np.tan(_FOV[1]/2)*2]).T

    oriImgRealPos=oriImgPixPos*imgRealMeasurement/resolution.T
    oriImgRealPos=np.dot(np.array([[np.cos(-roll),-np.sin(-roll)],[np.sin(-roll),np.cos(-roll)]]),oriImgRealPos)

    theta=np.arctan(oriImgRealPos[1]/d)
    phi=pitch+theta
    oriRealPos=np.array([oriImgRealPos[0]/(d/np.cos(theta))*(height/(np.sin(-phi))),height/np.tan(-phi)]).T

    pos=np.dot(np.array([[np.cos(-yaw),-np.sin(-yaw)],[np.sin(-yaw),np.cos(-yaw)]]),oriRealPos+np.array([x,y]).T)

    return pos.tolist()

def pos2pix(pose:list,FOV,pos:list):

    resolution=np.array(RESOLUTION)-1

    x,y,height,yaw,pitch,roll=pose

    yaw=yaw*0.017453292519943
    pitch=pitch*0.017453292519943
    roll=roll*0.017453292519943
    _FOV=np.array(FOV)*0.017453292519943

    d=height/np.sin(-pitch)
    imgRealMeasurement=np.array([d*np.tan(_FOV[0]/2)*2,d*np.tan(_FOV[1]/2)*2])

    _pos=np.dot(np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]),np.array([pos[0]-x,pos[1]-y]).T)

    phi=-np.arctan(height/_pos[1])
    theta=phi-pitch
    imgRealPos=np.array([_pos[0]/(height/(np.sin(-phi)*d/np.cos(theta))),d*np.tan(theta)]).T

    imgRealPos=np.dot(np.array([[np.cos(roll),-np.sin(roll)],[np.sin(roll),np.cos(roll)]]),imgRealPos)
    imgPixPos=imgRealPos/imgRealMeasurement*resolution.T

    pix=np.round([resolution[0]/2+imgPixPos[0],resolution[1]/2-imgPixPos[1]])

    if pix[0]<0 or pix[0]>resolution[0] or pix[1]<0 or pix[1]>resolution[1]:
        return None
    else:
        return pix.tolist()

def getIsOutOfFOV(pose1,FOV1,pos1:Position,pose2,FOV2,pos2:Position):
    if pos2pix(pose1,FOV1,pos2.tolist())==None and pos2pix(pose2,FOV2,pos1.tolist())==None:
        return True
    else:
        return False

#pos must be 3-d, return pix,isInFOV,isRealPix
def pos2pix_2(pose:list,inmtx,pos:list):

    x,y,height,yaw,pitch,roll=pose

    yaw=yaw*0.017453292519943
    pitch=pitch*0.017453292519943
    roll=roll*0.017453292519943

    rotation_mat=np.mat([[np.cos(-yaw),-np.sin(-yaw),0],[np.sin(-yaw),np.cos(-yaw),0],[0,0,1]]) \
                *np.mat([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]]) \
                *np.mat([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]]) \
                *np.mat([[1,0,0],[0,0,1],[0,-1,0]])

    exmtx=np.linalg.inv(np.r_[np.c_[rotation_mat,np.mat([[x],[y],[height]])],np.mat([0,0,0,1])])

    result=np.c_[inmtx,np.zeros([3,1])]*exmtx*np.mat([[pos[0]],[pos[1]],[pos[2]],[1]])

    if result[2,0]==0:
        return np.round([inmtx[0,2],inmtx[1,2]]).tolist(),False,False
    else:
        pix=np.round([result[0,0]/result[2,0],result[1,0]/result[2,0]])
        if result[2,0]<0:
            return pix.tolist(),False,False
        else:
            if pix[0]<0 or pix[0]>RESOLUTION[0] or pix[1]<0 or pix[1]>RESOLUTION[1]:
                return pix.tolist(),False,True
            else:
                return pix.tolist(),True,True

#pos will be 3-d, return pos,isRealPos
def pix2pos_2(pose:list,inmtx,pix:list,d=None,inv_inmtx=None):

    x,y,height,yaw,pitch,roll=pose

    yaw=yaw*0.017453292519943
    pitch=pitch*0.017453292519943
    roll=roll*0.017453292519943

    rotation_mat=np.mat([[np.cos(-yaw),-np.sin(-yaw),0],[np.sin(-yaw),np.cos(-yaw),0],[0,0,1]])\
                *np.mat([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]])\
                *np.mat([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]])\
                *np.mat([[1,0,0],[0,0,1],[0,-1,0]])

    T=-np.linalg.inv(rotation_mat)*np.mat([x,y,height]).T
    if not isinstance(inv_inmtx,np.matrix):
        inv_inmtx=np.linalg.inv(inmtx)

    if not d:
        if abs(np.sin(-pitch))<0.001:
            return [0,0,0],False
        else:
            d=height/np.sin(-pitch)

    center=rotation_mat*(inv_inmtx*d*np.mat([inmtx[0,2],inmtx[1,2],1]).T-T)
    z=center[2,0]

    posNorm=rotation_mat*(inv_inmtx*np.mat([pix[0],pix[1],1]).T-T)

    if posNorm[2,0]>=height:
        return [0,0,0],False
    else:
        pos=[x+(posNorm[0,0]-x)/(height-posNorm[2,0])*(height-z),y+(posNorm[1,0]-y)/(height-posNorm[2,0])*(height-z),z]
        return pos,True

def getIsOutOfFOV_2(pose1,mtx1,pos1:Position,pose2,mtx2,pos2:Position):
    if pos2pix_2(pose1,mtx1,pos2.tolist())[1]==False and pos2pix_2(pose2,mtx2,pos1.tolist())[1]==False:
        return True
    else:
        return False

if __name__ == '__main__':
    # nx, ny = [i for i in range(8)],[i for i in range(8)]
    # edge = {i:[j for j in range(8)] for i in range(8)}
    nx, ny = [i for i in range(12)],[i for i in range(12)]
    edge = {i:[j for j in range((i-2) if (i-2)>=0 else 0,(i+3) if (i+3)<12 else 12)] for i in range(12)}
    snx,sny,sedge=simplifyFromEntire(nx,ny,edge)
    print(snx,sny,sedge)
    print(nx,ny,edge)
    maxMatchNum,result=getAllMaxMatch(snx,sny,sedge)
    print(result)
    print(maxMatchNum)
    print(len(result))
    print(pos2pix([0,0,100,10,-80,0],[30,15],[5,5]))
    print(pix2pos([0,0,100,10,-80,0],[30,15],[5,5]))
    FOV=[30,15]
    MTX=np.mat([[1/(np.tan(FOV[0]/2/180*np.pi)/(RESOLUTION[0]/2)),0,(RESOLUTION[0]/2)],[0,1/(np.tan(FOV[1]/2/180*np.pi)/(RESOLUTION[1]/2)),(RESOLUTION[1]/2)],[0,0,1]])
    print(pos2pix_2([0,0,100,10,-80,0],MTX,[5,5,0]))
    print(pix2pos_2([100,200,300,10,-30,30],MTX,[0,0]))
    print(getIsOutOfFOV_2([0,0,10,10,-85,0],MTX,Position(0,0,0),[10,0,10,10,-85,0],MTX,Position(10,0,0)))