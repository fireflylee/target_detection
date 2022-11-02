# __Version__ = 7

from utilities import getAllMaxMatch,simplifyFromEntire,pos2pix_2,getIsOutOfFOV_2,Position,KalmanFilter
import json
from configs import *
import math

BASE_VERSION = 7

class Pool(list):
    def __contains__(self,x):
        for item in self:
            if x.id==item.id:
                return True
        return False

    def getItemById(self,id):
        for item in self:
            if id==item.id:
                return item
        return None

class Target:
    id=0
    pos=None
    timestamp=-1
    startTimestamp=-1
    sigma=0

    def __init__(self,id=0,pos=Position.zero(),sigma=ERROR_RANGE/3):
        self.id=id
        self.pos=pos
        self.sigma=sigma

    def getMaxPosError(self,timestampNow):  #暂时没考虑timestamp
        return 3*self.sigma                 #可改成 return 3*self.sigma+MAX_UGV_SPEED*(timestampNow-self.timestamp)*TIMESTAMP2SECOND

    def getIsNearby(self,other,timestampNow):
        return self.pos.getDistanceTo(other.pos)<self.getMaxPosError(timestampNow)+other.getMaxPosError(timestampNow)

    def getNearbyLevel(self,other,timestampNow):
        distance=self.pos.getDistanceTo(other.pos)
        err1=self.getMaxPosError(timestampNow)
        err2=other.getMaxPosError(timestampNow)
        if distance<err1 and distance<err2:
            return 1
        elif distance>err1 and distance>err2:
            return 3
        else:
            return 2

class UavTarget(Target):
    uav=None

    def __init__(self,id,pos,uav,sigma=ERROR_RANGE/3):
        super(UavTarget,self).__init__(id,pos,sigma)
        self.uav=uav
        self.timestamp=uav.timestamp
        self.startTimestamp=uav.timestamp

    def update(self,pos,sigma=None):
        self.pos=pos
        if sigma:
            self.sigma=sigma
        self.timestamp=self.uav.timestamp

class FusionTarget(Target):
    uavTargetPool:Pool
    state='Null'
    fusionPos=None
    fusionSpd=None
    kf=None
    isNearlyLost=False
    lastTimestamp=-1

    kfTimeoutTick=KF_TIMEOUT_TICK_MAX

    def __init__(self,id,targetPool,timestamp):
        super(FusionTarget,self).__init__(id)
        self.uavTargetPool=targetPool
        if len(targetPool)>0:
            self.__update(timestamp)
            self.startTimestamp=timestamp
        self.fusionPos=self.pos.clone()
        self.fusionSpd=[0,0]
        self.kf=KalmanFilter(self.pos,KF_P,KF_R)

    def __contains__(self,x):
        if isinstance(x,Uav):
            for item in self.uavTargetPool:
                if x.id==item.uav.id:
                    return True
        elif isinstance(x,UavTarget):
            for item in self.uavTargetPool:
                if x.uav.id==item.uav.id and x.id==item.id:
                    return True
        return False

    def remove(self,uav,trueRemove=False):   #return True if target is TRULY removed else return False, updated in V6
        for target in self.uavTargetPool:
            if target.uav.id==uav.id:
                if len(self.uavTargetPool)>1:
                    self.uavTargetPool.remove(target)
                    self.__update(uav.timestamp)
                    return True
                else:
                    if not self.isNearlyLost and not trueRemove:
                        self.isNearlyLost=True
                        self.lastTimestamp=uav.timestamp
                        self.__update(uav.timestamp)
                    elif trueRemove or uav.timestamp>self.lastTimestamp+MAX_LOST_DELAY/TIMESTAMP2SECOND/1000:
                        self.uavTargetPool.remove(target)
                        self.__update(self.timestamp)
                        return True
                return False
        return False

    def add(self,uavTarget,timestamp=None):
        if self.state=='Lost':
            self.startTimestamp=uavTarget.startTimestamp
        if self.isNearlyLost:
            self.isNearlyLost=False
            self.uavTargetPool.clear()
        self.uavTargetPool.append(uavTarget)
        self.__update(uavTarget.uav.timestamp if not timestamp else timestamp)

    def update(self,uav):
        for target in self.uavTargetPool:
            if target.uav.id==uav.id:
                if self.isNearlyLost:
                    self.isNearlyLost=False
                self.__update(uav.timestamp)
                return

    def __update(self,timestampNow):
        if len(self.uavTargetPool)>0:

            #高斯加权
            self.sigma=math.sqrt(1/(sum([1/target.sigma**2 for target in self.uavTargetPool])))
            self.pos=sum([target.pos*target.sigma**2 for target in self.uavTargetPool])/sum([target.sigma**2 for target in self.uavTargetPool])

            #self.pos=sum([target.pos for target in self.uavTargetPool])/len(self.uavTargetPool)    #融合,由于误差不变,此处为直接平均,maxPosError不变
            
            if self.isNearlyLost:
                self.state='Nearly_Lost'
            else:
                self.state='Active'
                if self.timestamp>=0:
                    if timestampNow>self.timestamp:
                        if self.kfTimeoutTick<KF_TIMEOUT_TICK_MAX:
                            self.kfTimeoutTick=KF_TIMEOUT_TICK_MAX
                        self.kf.update(self.pos,(timestampNow-self.timestamp)*TIMESTAMP2SECOND)
                        self.fusionPos.x=self.kf.X[0,0]
                        self.fusionPos.y=self.kf.X[1,0]
                        self.fusionPos.z=self.kf.X[2,0]
                        if self.kf.X[3,0]**2+self.kf.X[4,0]**2<=MAX_UGV_SPEED**2:
                            self.fusionSpd[0]=self.kf.X[3,0]
                            self.fusionSpd[1]=self.kf.X[4,0]
                        else:
                            self.fusionSpd[0]=0
                            self.fusionSpd[1]=0
                    else:
                        if self.kfTimeoutTick>0:
                            self.fusionPos.x=self.pos.x
                            self.fusionPos.y=self.pos.y
                            self.fusionPos.z=self.pos.z
                            self.kfTimeoutTick-=1
                        else:
                            self.fusionPos.x=self.pos.x
                            self.fusionPos.y=self.pos.y
                            self.fusionPos.z=self.pos.z
                            self.fusionSpd[0]=0
                            self.fusionSpd[1]=0
        else:
            self.state='Lost'
        
        if timestampNow>self.timestamp:
            self.timestamp=timestampNow

class Uav:
    id=0
    pose=None
    base=None
    
    timestamp=-1
    latestTarget=0
    uavTargetPool=Pool()
    newTargetTupleList=[]
    trackingIdList=[]

    def __init__(self,id,base=None):
        self.id=id
        self.base=base
        self.pose=None
        self.uavTargetPool=Pool()
        self.newTargetTupleList=[]
        self.trackingIdList=[]

    
    def checkAlive(self):
        if self.timestamp<0:
            return False
        else:
            return True

    def update(self,message):
        isNewTargetFound=False
        self.timestamp=message['timestamp']
        self.pose=message['pose']
        targets=message['targets']
        if self.base:
            #删去丢失的目标
            for fusion in self.base.fusionTargetPool:
                if self in fusion:
                    for uavTarget in fusion.uavTargetPool:
                        if uavTarget.uav.id==self.id:
                            if not any(target['id']==uavTarget.id for target in targets):
                                if uavTarget in self.uavTargetPool:
                                    self.uavTargetPool.remove(uavTarget)
                                    if uavTarget.id in self.trackingIdList:
                                        self.trackingIdList.remove(uavTarget.id)
                                    #if not self.base.isUpdated:
                                    #    self.base.isUpdated=True
                                isRemove=fusion.remove(self)

                                #检查融合目标重合并规则
                                if isRemove and fusion.state=='Active': #此处state必然为Active
                                    reunionOffsets=[]
                                    reunionFusions=[]
                                    for nextFusion in self.base.fusionTargetPool:
                                        if nextFusion is not fusion and nextFusion.state!='Lost':
                                            if fusion.getIsNearby(nextFusion,self.timestamp):
                                                if not any(target.uav in nextFusion for target in fusion.uavTargetPool):    #关联集合交集为空集
                                                    reunionOffsets.append(fusion.pos.getDistanceTo(nextFusion.pos))
                                                    reunionFusions.append(nextFusion)
                                    while len(reunionOffsets)>0:
                                        index=reunionOffsets.index(min(reunionOffsets))
                                        reunionOffsets.pop(index)
                                        nextFusion=reunionFusions.pop(index)
                                        #检查临近等级
                                        if fusion.getNearbyLevel(nextFusion,self.timestamp)>=NEARBY_LEVEL:
                                            if not any(pos2pix_2(target.uav.pose,MTXS[target.uav.id-1],nextFusion.pos.tolist())[1] for target in fusion.uavTargetPool):
                                                continue
                                        #重合并
                                        if fusion.startTimestamp>nextFusion.startTimestamp or fusion.startTimestamp<0:
                                            src=fusion
                                            dst=nextFusion
                                        else:
                                            src=nextFusion
                                            dst=fusion
                                        # print([(t.uav.id,t.id) for t in src.uavTargetPool])
                                        # print([(t.uav.id,t.id) for t in dst.uavTargetPool])
                                        while len(src.uavTargetPool)>0:
                                            target=src.uavTargetPool[0]
                                            dst.add(target,target.timestamp)
                                            src.remove(target.uav,trueRemove=True)
                                        break

                            break

            #甄别新目标并判断起始条件
            for i in range(len(self.newTargetTupleList)-1,-1,-1):
                newTargetTuple=self.newTargetTupleList[i]
                if not any(newTargetTuple[0]==target['id'] for target in targets):              #删除不满足起始条件的目标
                    self.newTargetTupleList.pop(i)
                elif self.timestamp>newTargetTuple[1]+TRACE_START_DELAY/TIMESTAMP2SECOND/1000:  #接受满足起始条件的目标
                    self.trackingIdList.append(newTargetTuple[0])
                    #if not self.base.isUpdated:
                    #    self.base.isUpdated=True
                    self.newTargetTupleList.pop(i)

            for i in range(len(targets)-1,-1,-1):
                target=targets[i]
                if not target['id'] in self.trackingIdList:                                     #新目标开始起始条件判断并剔除出当前targets
                    targets.pop(i)
                    if not any(newTargetTuple[0]==target['id'] for newTargetTuple in self.newTargetTupleList):
                        self.newTargetTupleList.append((target['id'],self.timestamp))

            if len(targets)==0:
                return
            
            #更新已有目标或添加新目标
            for target in targets:
                uavTarget=self.uavTargetPool.getItemById(target['id'])
                if uavTarget:   #已有目标更新
                    uavTarget.update(Position(target['x'],target['y'],target['z'] if 'z' in target.keys() else 0), target['sigma'] if 'sigma' in target.keys() else ERROR_RANGE/3)
                    for fusion in self.base.fusionTargetPool:
                        if uavTarget in fusion:
                            fusion.update(self)
                            break
                else:           #新目标
                    if self.latestTarget<target['id']:
                        self.latestTarget=target['id']
                    isExisted=False
                    for fusion in self.base.fusionTargetPool:
                        #新目标是当前融合结果中存在的目标,该目标可能是刚丢失的目标但再次被捕获
                        if len(fusion.uavTargetPool)==1 and fusion.state=="Nearly_Lost" and fusion.uavTargetPool[0].id==target['id'] and fusion.uavTargetPool[0].uav.id==self.id:
                            targetInFusion=fusion.uavTargetPool[0]
                            targetInFusion.pos=Position(target['x'],target['y'],target['z'] if 'z' in target.keys() else 0)
                            targetInFusion.sigma=target['sigma'] if 'sigma' in target.keys() else ERROR_RANGE/3
                            targetInFusion.timestamp=self.timestamp
                            self.uavTargetPool.append(targetInFusion)
                            fusion.update(self)
                            isExisted=True
                            break
                    if not isExisted:
                        self.uavTargetPool.append(UavTarget(target['id'],Position(target['x'],target['y'],target['z'] if 'z' in target.keys() else 0),self,target['sigma'] if 'sigma' in target.keys() else ERROR_RANGE/3))
                        isNewTargetFound=True
            
            #重匹配
            if isNewTargetFound:
                self.base.refusionBy(self)

class Base:

    uavPool:Pool
    fusionTargetPool:Pool
    
    timestamp=-1
    latestTarget=0

    #isUpdated=False

    def __init__(self):
        self.uavPool=Pool([Uav(i+1,self) for i in range(MAX_UAV_NUM)])
        self.fusionTargetPool=Pool()
        #self.isUpdated=True

    def update(self,messageStr):
        try:
            message=json.loads(messageStr)
            timestamp=message['timestamp']
            uavId=message['uav']
            uav=self.uavPool.getItemById(uavId)
            if uav.timestamp<timestamp:
                uav.update(message)
                if self.timestamp<timestamp:
                    self.timestamp=timestamp
        except Exception as err:
            print(err)

    def fusionBy(self,uav,excludeIdList=[]):
        matched={target.id:(1 if target.id in excludeIdList else 0) for target in uav.uavTargetPool}
        newFusionTargetPool=Pool()
        nearbyEdge={}           #所有相邻关系集合,方向从target到fusion
        nearbyFusionIds=[]      #所有fusion中的相邻点编号集合
        nearbyTargetIds=[]      #所有uavTarget中的相邻点编号集合

        #分离出不相邻点
        for target in uav.uavTargetPool:
            if not matched[target.id]:
                isNearby=False
                for fusion in self.fusionTargetPool:
                    if (fusion.state=='Active' and not uav in fusion) or fusion.state=='Nearly_Lost':    #不匹配丢失目标和保留目标
                        if target.getIsNearby(fusion,self.timestamp):
                            isNearby=True
                            if not target.id in nearbyEdge:
                                nearbyEdge[target.id]=[fusion.id]
                            else:
                                nearbyEdge[target.id].append(fusion.id)
                            if not fusion.id in nearbyFusionIds:
                                nearbyFusionIds.append(fusion.id)
                if not isNearby:
                    if ENABLE_INDEX_REUSE and any(fusion.state=='Lost' for fusion in self.fusionTargetPool):
                        for fusion in self.fusionTargetPool:
                            if fusion.state=='Lost':
                                fusion.add(target)
                                break
                    else:
                        self.latestTarget+=1
                        newFusionTargetPool.append(FusionTarget(self.latestTarget,Pool([target]),target.uav.timestamp))
                    matched[target.id]=1

        nearbyTargetIds=list(nearbyEdge.keys())

        def getMatchError(matches,withOffset):
            offset=(sum([uav.uavTargetPool.getItemById(u).pos-self.fusionTargetPool.getItemById(v).pos for (u,v) in matches])/len(matches)) if withOffset else Position.zero()
            return sum([self.fusionTargetPool.getItemById(v).pos.getSquareDistanceTo(uav.uavTargetPool.getItemById(u).pos-offset) for (u,v) in matches]) + abs(offset)

        #重复寻找相邻点集合直至所有目标均被分配
        while len(nearbyTargetIds)>0:
            nx,ny,edge=simplifyFromEntire(nearbyTargetIds,nearbyFusionIds,nearbyEdge)
            maxMatchNum,result=getAllMaxMatch(nx, ny, edge)
            bestError=-1
            bestMatches=None
            for matches in result:
                    error=getMatchError(matches,maxMatchNum>=2)
                    if bestError<0 or error<bestError:
                        bestError=error
                        bestMatches=matches
            for match in bestMatches:
                targetId,fusionId=match
                target=uav.uavTargetPool.getItemById(targetId)
                fusion=self.fusionTargetPool.getItemById(fusionId)
                if fusion.state=='Active':
                    if target.getNearbyLevel(fusion,uav.timestamp)>=NEARBY_LEVEL:
                        if all(getIsOutOfFOV_2(uav.pose,MTXS[uav.id-1],target.pos,each.uav.pose,MTXS[each.uav.id-1],each.pos) for each in fusion.uavTargetPool):
                            # print(fusion.id)
                            # print(fusion.pos)
                            # print(target.pos)
                            continue
                self.fusionTargetPool.getItemById(fusionId).add(uav.uavTargetPool.getItemById(targetId))
                matched[targetId]=1

        #将剩余uavTarget直接分配为新的fusionTarget
        for target in uav.uavTargetPool:
            if not matched[target.id]:
                if ENABLE_INDEX_REUSE and any(fusion.state=='Lost' for fusion in self.fusionTargetPool):
                    for fusion in self.fusionTargetPool:
                        if fusion.state=='Lost':
                            fusion.add(target)
                            break
                else:
                    self.latestTarget+=1
                    newFusionTargetPool.append(FusionTarget(self.latestTarget,Pool([target]),target.uav.timestamp))

        #合并
        self.fusionTargetPool+=newFusionTargetPool

    def refusionBy(self,uav):
        #删除分量时保留仅由此uav融合生成的目标
        excludeIdList=[]
        for fusion in self.fusionTargetPool:
            if uav in fusion:
                if len(fusion.uavTargetPool)>1:
                    fusion.remove(uav)
                else:
                    excludeIdList.append(fusion.uavTargetPool[0].id)

        self.fusionBy(uav,excludeIdList)