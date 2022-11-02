import numpy as np

MAX_UAV_NUM=12          #最大UAV数
MAX_UGV_NUM=4           #最大UGV数
MAX_UGV_SPEED=10        #最大UGV速度,m/s

REAL_TIME_FACTOR=1.0    #时间因子
RESOLUTION=[1920,1080]  #分辨率
ERROR_RANGE=25          #最大误差,m
MAX_LOST_DELAY=2000     #最大丢失后保留时间,ms
TRACE_START_DELAY=650   #目标起始保留时间,ms
NEARBY_LEVEL=3          #临近等级,用于判断是否需要检查视野重叠
                        # 1:永远检查视野重叠,
                        # 2:目标不相互重叠则检查视野重叠,
                        # 3:目标不重叠则检查视野重叠,
                        # 4:不考虑检查视野重叠,
                        #其中,目标重叠表示两目标均在对方允许的误差范围内,不重叠表示均在误差范围外但可能误差范围存在交集
                        #数值越大相邻程度越低
ENABLE_INDEX_REUSE=True #允许重复使用目标已经丢失的编号
TIMESTAMP2SECOND=0.001  #时间戳变化1对应的实际秒数
KF_P=np.eye(6)*100      #Kalman Filter P阵
KF_R=np.eye(3)*0.1      #Kalman Filter R阵
KF_TIMEOUT_TICK_MAX=6   #Kalman Filter 在长时间未收到新数据时后结果保留数据帧数

#''' <--- 注释此行选择场景
# Gazebo

# Gazebo 12
# MAX_UAV_NUM=12
# REAL_TIME_FACTOR=0.8
# MAX_UGV_NUM=4

# Gazebo 24
# MAX_UAV_NUM=24
# REAL_TIME_FACTOR=0.2
# MAX_UGV_NUM=12
# TRACE_START_DELAY=250

# U3d
MAX_UAV_NUM=12
MAX_UGV_NUM=4
TRACE_START_DELAY=650
MAX_LOST_DELAY=4000

RESOLUTION=[1920,1080]  #模拟分辨率
FOV_H=30                #模拟视场角,横向,deg
FOV=[FOV_H,FOV_H/RESOLUTION[0]*RESOLUTION[1]]             #模拟视场角,deg
#模拟内参矩阵,由视场角和分辨率计算获得,假设所有uav均相同
MTX=np.mat([[1/(np.tan(FOV[0]/2/180*np.pi)/(RESOLUTION[0]/2)),0,(RESOLUTION[0]/2)],[0,1/(np.tan(FOV[1]/2/180*np.pi)/(RESOLUTION[1]/2)),(RESOLUTION[1]/2)],[0,0,1]])
MTXS=[MTX for _ in range(MAX_UAV_NUM)]
KF_P=np.eye(6)*100
KF_R=np.eye(3)*0.1

'''
# 四旋翼

MAX_UAV_NUM=3
RESOLUTION=[1920,1080]
MTXS=[np.mat([[1248.739176,0.000000,979.348146],[0.000000,1332.824269,510.280853],[0.000000,0.000000,1.000000]]), \
      np.mat([[1237.680132,0.000000,969.903552],[0.000000,1321.087868,523.339337],[0.000000,0.000000,1.000000]]), \
      np.mat([[1231.535153,0.000000,935.769547],[0.000000,1314.163167,518.691510],[0.000000,0.000000,1.000000]])]

#'''

INV_MTXS=[np.linalg.inv(MTX) for MTX in MTXS]

MAX_LOST_DELAY=int(MAX_LOST_DELAY/REAL_TIME_FACTOR)
TRACE_START_DELAY=int(TRACE_START_DELAY/REAL_TIME_FACTOR)
TIMESTAMP2SECOND*=REAL_TIME_FACTOR

SAVE_DATA=True          #存储原始数据
DATA_PORT=5000          #检测结果数据端口
REAL_PORT=6000          #实际位置数据端口
VISUALIZATION_PORT=7000 #可视化数据端口