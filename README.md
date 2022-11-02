#   说明

    程序实现了一个基于最大匹配数与最小几何误差假设的多目标融合方法。
    
    其中，最大匹配数假设指：来自不同源的目标群中，融合结果包含的目标个数应为两目标群相邻目标的最大匹配数，和剩余不相邻及未匹配的目标数之和；最小几何误差假设指：相邻目标间的匹配准则选择几何误差最小的匹配。上述假设源自于UAV基于图像的目标感知与定位原理，遵从UAV图像感知模型的运行特点与误差特性，并不一定适用于其他一般多目标融合场景。
    与传统多目标融合算法相比，该算法放弃了对一次轨迹的滤波过程，最大限度保留图像测量的误差特性，使用大量逻辑判断作为匹配准则，严格来说算法逻辑并不严谨，属于一种非常朴素的多目标融合算法。
    
    程序使用Python编写（Python3.x 标准），在Windows和Linux环境下均可运行，通信以UDP广播形式进行，内容为Json报文。如特殊需要可将程序运行于机载端以降低传输延迟。


#   输入、输出与算法核心想法

##      输入

        来自UAV的检测与跟踪结果，包含有时间戳、UAV编号、UAV机体位姿（x,y,z,yaw,pitch,roll）、感知目标池，其中感知目标池包含各目标的跟踪编号、位置（xy[z]），跟踪编号为UAV检测完成后对目标进行多目标跟踪获得的编号，原则上编号单调增，且各目标不复用编号。
        输入速率为5Hz/UAV，其中5Hz为各无人机数据的发送频率。


##      输出

        多目标融合结果，即融合目标池，包含各融合目标的融合编号、位置（xyz）、融合信息源与跟踪状态，其中融合编号与融合目标一一对应，可按情况选择是否复用编号，原则上编号应与对应实际目标实现绑定且不发生变化；融合信息源包含该融合目标包含的全部无人机源及其对应的跟踪编号；跟踪状态分为正常跟踪（Active）、丢失（Lost，所有无人机丢失对该目标的观测）、即将丢失（Nearly-Lost，最后一个无人机丢失对该目标的观测，对融合结果暂时保留一定时间，若时间过长则进入Lost状态）。
        输出速率与输入速率一致。


##      算法核心想法

        考虑到输入特性，输入算法的目标信息已预先完成单UAV的检测与多目标跟踪，即一次轨迹，则融合过程可认为仅发生在新目标出现时或旧目标消亡时。
        其中，新目标出现时或漏检测后重新出现时，由最大匹配数假设，首先将该无人机的目标池从融合目标池中剔除，并以当前目标位置关系重新进行融合，对应base.Base.refuionBy与base.Base.fusionBy函数，该过程实现了对错误匹配的动态纠错，同时融合过程对融合编号几乎没有影响；旧目标消亡时，由与可能存在的误检测问题，未防止误检测的目标被独立分配为多个融合目标，需要对旧目标消亡进行判断，若消亡目标为误检测目标则需要进行重合并，该过程在base.UAV.update函数中有体现。
        除此之外，只需对已匹配的目标进行数据更新即可。


#   文件结构与关键内容

##      /fusion
        根目录

###         /2.22.pptx
            描述了算法的基本想法和运行流程，比较抽象，里面提到的想法在算法中也不是全部都实现了

###         /4.26.pptx
            描述了仿真环境及数据流

###         /dev_log.txt
            开发存档

###         /base.py
            融合中心的实现代码，包含了对池、目标、UAV及融合中心的类定义，通过实例化Base即可实现一个融合中心的功能，并通过Base.update函数实现UAV数据的输入，更新融合结果

                class Pool：基于list实现了一个池对象的定义，并定义了通过id查找内容的方法

                class Target：定义了一个目标对象的基类，并定义了判断相邻的相应方法
                    id：目标编号
                    timestamp：时间戳
                    pos：目标位置
                    sigma：误差高斯分布等效标准差

                class UavTarget：基于Target定义了一个UAV检测目标
                    uav：该目标所属的UAV对象

                class FusionTarget：基于Target定义了一个融合目标，并定义了向融合目标中融合、删除、更新UAV检测目标的方法
                    state：融合目标状态，根据融合状态具体分为：
                        Active:当前融合目标正被至少一个UAV跟踪
                        Nearly-Lost:当前融合目标中唯一一个在跟踪的UAV丢失了对该目标的跟踪，目标将保留一段时间并进入Lost状态
                        Lost:所有UAV丢失对该目标的跟踪
                        Null:无效值
                    uavTargetPool：该融合目标的源UAV检测目标集合，及融合前的UAV检测目标集合
                    kf：卡尔曼滤波器
                    fusionPos：由滤波器滤波后获得的位置，注意此位置不参与融合过程，仅用于输出平滑

                class UAV:定义了一个UAV，并定义了从实际UAV接收数据并更新UAV目标池的方法
                    id：UAV编号
                    timestamp：时间戳
                    pose：UAV位姿
                    base：UAV对应融合中心对象，用于内部调用
                    uavTargetPool：UAV检测目标池，包含来自该UAV的全部检测目标，注意与FusionTarget的uavTargetPool意义不同
                    trackingIdList与newTargetTupleList：用于实现对新目标的序贯检验，即起始判断
                    def update：由实际UAV接收的数据更新UAV目标池的方法，在更新完UAV目标池后将该目标池按情况提交Base并实现融合

                class Base：定义了一个融合中心，并定义了从各UAV更新融合目标池的方法
                    uavPool：UAV池，包含全部UAV对象
                    fusionTargetPool：融合目标池，包含融合后的目标集合
                    def update：处理来自UAV的数据，并根据UAV id交由对应UAV对象处理
                    def fusionBy：核心方法，实现了一次融合过程
                        def getMatchError：在匹配对中获得匹配误差（形状误差）
                    def refusionBy：在fusionBy前从融合目标池中删去待融合无人机的全部检测目标分量

###         /configs.py
            包含算法运行时的必要参数

                MAX_UAV_NUM：最大UAV数，该数字比当前环境中UAV数大即可
                MAX_UGV_NUM：最大UGV数
                MAX_UGV_SPEED：最大UGV速度，该值用于限制卡尔曼滤波器获得的最大速度
                REAL_TIME_FACTOR：时间因子，在实际实验中取1.0，在仿真中取gazebo的时间因子
                ERROR_RANGE：最大误差，即目标实际位置与测量位置的距离不超过此值，用于判断相邻关系，注意此值在算法本应可变，但考虑到没有实际测试，此处暂时取一固定值
                MAX_LOST_DELAY：丢失后保留时间，在目标丢失后对融合目标保留一段时间，以便目标被重新捕获
                TRACE_START_DELAY：目标起始保留时间，在UAV检测目标第一次出现时至开始处理预留的等待时间，在此期间必须保持目标持续出现，否则认为是虚警信号
                NEARBY_LEVEL：临近等级，参照configs.py中的注释
                ENABLE_INDEX_REUSE：允许融合编号复用，为True时允许新的融合目标使用已丢失的目标
                TIMESTAMP2SECOND：实际变化时间（单位s）与时间戳变化量的比值
                MTXS：UAV相机矩阵列表，顺序与UAV编号一致
                SAVE_DATA：是否保存数据记录，为True时会在/fusion/backup/datas中留下数据记录，用于数据复现

###         /utilities.py
            包含了程序运行时需要的一些计算方法和类定义

                def round_floats：用于对dict中浮点数精度做出限制，以压缩str(dict)大小
                def getProgressString：用于获取一个进度条字符串，默认以百分比显示
                def tailMethod4timedelta：用于getProgressString中以时间显示进度
                class Position：定义了一个三位坐标，并实现了基本运算和求距离运算等
                class KalmanFilter：定义了一个卡尔曼滤波器，其中使用恒速模型（CV模型）建模
                def getMaxMatchNum：获取二分图最大连接数，原理参考匈牙利算法
                def getAllMaxMatch：获取二分图中连接对数为最大连接数的全部连接对集合的集合
                def simplifyFromEntire：从二分图中分理出一个与剩余部分不相连的子二分图，用于对二分图剪枝
                def pix2pos：[已弃用] 从像素坐标变换为实际坐标 
                def pos2pix: [已弃用] 从实际坐标变换为像素坐标
                def getIsOutOfFOV：[已弃用] 用于分别出现在两UAV视野边界处的相邻目标，判断各目标是否均在对方的视野外
                def pix2pos_2：从像素坐标变换为实际坐标
                def pos2pix_2: 从实际坐标变换为像素坐标
                def getIsOutOfFOV_2：用于分别出现在两UAV视野边界处的相邻目标，判断各目标是否均在对方的视野外

###         /server.py
            包含了用于通信的服务器，包含TCP与UDP形式，通过传入callback即可实现对收到数据的回调过程

###         /main.py
            实现一个融合过程的主函数，其过程包含有：从UAV收到数据、交由Base完成融合并最终发送给可视化端与决策中心

                dst：与决策中心通信的目的地址与端口，按实际值修改即可

                请务必使用Ctrl-C结束程序，否则可能造成数据存储时条目长度不一致的问题

###         /visualization.py
            可视化程序，用于显示全部UAV、UAV检测目标、实际目标与融合目标

                UAV：以编号字符串显示
                UAV检测目标：以X显示，颜色与检测到该目标的UAV一致
                实际目标：以正方形显示
                融合目标：以〇显示表示正常跟踪，以*显示表示即将丢失，以三角形表示已经丢失
                UAV视野：以梯形框显示，颜色与对应UAV一致
                UAV相机光轴：以虚线显示，与地面交点以+显示，颜色与对应UAV一致

                改变x_lim与y_lim可改变绘制范围，可尝试注释绘制其中语句以改变绘制内容

###         /visualization3d.py
            可视化程序的3d版本，显示内容与visualization.py类似

                DRAW_TRAJ：控制是否绘制轨迹
                TRAJ_LEN：轨迹保留的线段数，若为-1则为保留全部轨迹

###         /visualizationCamera.py
            虚拟UAV摄像机的实际画面，将目标投影在画面中，并标出画面北向、云台移动方向和目标锁定指示，用于在实际实验条件下模拟出摄像机画面，避免传输实际画面造成的带宽占用

###         /reproduce.py
            数据复现程序，可复现由main.py保存的原始数据

                path：数据文件夹路径
                tail：数据文件名结尾部分
                SEND_UDP_DIRECTLY：是否直接发送UDP数据
                    若SEND_UDP_DIRECTLY=True则此时等同于各UAV直接发送数据，可用于模拟实验现场的通信过程，此时需执行main.py
                    若SEND_UDP_DIRECTLY=False则此时由程序本身调用Base并一次性完成全部数据的读取和处理，并模仿main.py发送融合结果
                
                SEND_UDP_DIRECTLY=False时可通过键盘控制数据的发送速度、快进、暂停与重复等

###         /main_review.py
            [已弃用] 用于以二维可视化图窗形式复现数据

###         /ugv_broadcaster.py
            UGV坐标广播程序，用于实机实验时接收UGV RTK发送的坐标数据并转为实际坐标并传给可视化程序

                其中，cars为UGV对象car的集合，其中经纬高坐标为UGV处于起点位置的对应坐标，将UGV摆放至起点后用收到的经纬高坐标替换该值即可实现UGV原点标定

###         /ntp.py
            对时程序，用于实机实验时对各UAV提供统一的时间起点，实现UAV对时

                MODE：模式选择，UAV端为订阅者，PC端为发布者，UAV端需以root权限运行

                该程序使用jsonMessage格式通信，具体参考/jsonMessage

###         /jsonMessage
            包含有json通信的收发器和数据模板的类，有Python2/3及cpp版本的实现

####            /JsonMessageBase.py
                定义了json报文的基类，通过继承该基类并重写对应方法实现自定义的json报文数据，参考NTPMessage或TrackMessage

####            /JsonReader.py
                定义了json报文接收器，通过设置数据端口和json报文对象即可自动接收json数据并填充至该对象中

####            /JsonSender.py
                定义了json报文发送器，通过设置目的IP、数据端口和json报文对象，通过sendData函数即可发送该对象中的数据

###         /simulator
            使用给定误差形式模拟UAV及目标的纯数值仿真，由Matlab编写，运行main.py后运行simulator_2.m即可，仿真参数参照脚本中注释修改即可，建议使用gazebo仿真更真实一点，这版程序太老了

###         /traj_fusion_2d
            由传统航迹融合改的融合方法，使用UKF与KF滤波并以马氏距离作为匹配准则，由Matlab编写，然而参数太过于依赖人工调节，并不好用，考虑作为备份算法

###         /backup
            备份文件

####            /datas
                程序运行时保留的原始数据，由configs.py中SAVE_DATA控制产生

####            /codes
                旧版代码备份


#   对象关系、主要成员与整体组成结构

    base:Base                                               融合中心对象，实例获得一个具有融合结果更新功能的对象，实现对每条数据报文逐条融合
        uavPool:Pool<Uav>                                   无人机（Uav）池，包含有任务包含的所有无人机
            uav:Uav                                         无人机对象实例，对应各实际无人机，实例获得一个具有数据更新功能的对象，实现对报文所属无人机的观测数据进行更新，并根据情况判断是否交由Base进行融合或直接更新
                id:int                                      无人机编号
                pose:list                                   无人机位姿
                base:Base                                   父对象引用，此处只是用来调用父对象的融合函数
                uavTargetPool=Pool<UavTarget>               无人机当前所感知到所有地面目标（UavTarget）的目标池，池中包含当前无人机所感知得到的且满足起始条件（持续观察一段时间）的地面目标，表征为稳定地面目标
                    uavTarget:Target                        无人机所感知得到的单个地面目标对象实例，对应该无人机视野内的各地面目标，实例获得一个具有位置更新功能的对象
                        uav:Uav                             无人机对象引用，用于标记该无人机对象所对应的无人机，即从uavTarget获得其所属uav
                        id:int                              继承自Target，表征为当前无人机对该目标赋予的跟踪编号
                        pos:Position                        继承自Target，表征为该地面目标的世界坐标位置
                        sigma:float                         继承自Target，表征为该地面目标测量误差的等效正态分布标准差
        fusionTargetPool:Pool<FusionTarget>                 融合对象（FusionTarget）池，包含融合结果中所有的融合对象
            fusion:FusionTarget                             融合对象实例，对应各融合后目标，实例获得一个具有更新（update指定uav，进行丢失判断，并最终由__update计算更新后融合位置）、
                                                            添加（由Base融合方法控制将uavTarget加入self.uavTargetPool中）、删除（从self.uavTargetPool中删去一个关联）功能的对象
                uavTargetPool:Pool<UavTarget>               被关联到该融合目标的地面目标（UavTarget）的目标池，池中包含有来自不同无人机（Uav）的地面目标
                    uavTarget:Target                        定义同上述uavTarget，此处是对uav中uavTatgetPoll中uavTarget的引用
                id:int                                      继承自Target，表征为融合目标编号
                pos:Position                                继承自Target，表征为融合目标位置
                sigma:float                                 继承自Target，表征为该融合目标位置误差的等效正态分布标准差
                kf:KalmanFilter                             卡尔曼滤波器对象，用于传递给决策时对目标进行滤波
                fusionPos:Position                          滤波后位置
                fusionSpd:list                              滤波器估计的目标移动速度


#   算法运行流程

    1. 接收一条来自无人机的报文并交由base处理
    2. base将报文递交由对应无人机uav处理
    3. 无人机更新姿态、时间戳、地面目标等信息
        3.1. 检查是否存在丢失目标，若有则从融合结果中尝试删去丢失目标（此处若处于丢失后保留时间中，则无人机本地暂时保存该目标对象，用于当目标对象被再次捕获时调用）
        3.2. 若删去目标，则进行重合并判断，检查是否需要对多个相邻融合目标进行合并
        3.3. 对新出现的目标进行起始判断，满足起始判断的目标则添加入无人机目标池中
        3.4. 对无人机目标池内所有目标更新位置信息，并更新对应所属融合目标的位置信息
        3.5. 若出现新目标则通过base调用对该无人机进行融合，否则跳转至步骤6
    4. base删去所有融合目标中来自该无人机的地面目标，即融合目标池中的融合目标均不包含来自该无人机的分量部分
    5. base将融合目标池与该无人机目标池中的目标进行融合
        5.1 分离出与融合目标池中所有目标均不构成相邻关系的目标，直接分配为新的融合目标
        5.2 对剩余目标构造相邻关系二分图
        5.3 简化二分图，得到一个简化后的二分图和剩余部分，使简化后的二分图与剩余部分没有关联关系
        5.4 获得简化二分图的全部关联可能的集合，并遍历该集合获得一个误差最小的关联可能，并将产生关联的地面目标添加进对应的融合目标
        5.5 重复5.4，直至全部关联完成
        5.6 将剩余的地面目标直接分配为新的融合目标
        5.6 将新增的融合目标添加入融合目标池中
    6. 将base中的融合结果打包并递交至决策中心
    

#   执行顺序

    以Windows环境为例，Linux环境与之类似

##      正常执行

        1.设置参数
            修改configs.py选择当前实验或仿真时对应使用的参数，修改或注释应对不同场景的设置参数，并根据需要设置SAVE_DATA

        2.启动UGV坐标广播程序   [仅实机实验时使用，仿真中UGV坐标广播由仿真程序实现]
            在新cmd中执行python ugv_broadcaster.py，并将UGV放置在原点时设置cars中对应car的起始经纬高坐标
            该坐标仅用于可视化，不参与融合过程

        3.UAV对时
            在各UAV终端上运行sudo python3 ntp.py，其中模式选择订阅器模式
            在新cmd中执行python ntp.py，其中模式选择发布器
            在完成对时后关闭UAV终端程序及cmd即可

        4.启动可视化场景界面
            修改visualization.py/visualization3d.py中的场景参数
            在新cmd中执行python visualization.py / python visualization3d.py
            若在Linux中出现画面占据最上层情况，点击Configure subplots或其他按钮弹出设置弹出并关闭即可解决，原因不明。

        5.启动虚拟相机界面
            在新cmd中执行python visualizationCamera.py
        
        6.启动融合程序
            在新cmd中执行python main.py，结束时务必以Ctrl-C结束，否则会出现保存的数据长度不一致的问题


##      数据复现

        1.设置参数
            修改configs.py选择当前实验或仿真时对应使用的参数，修改或注释应对不同场景的设置参数，并设置SAVE_DATA为False

        2.启动复现程序
            修改reproduce.py中path和tail选择复现数据，并设置SEND_UDP_DIRECTLY
            若SEND_UDP_DIRECTLY=True则此时等同于各UAV直接发送数据，可用于模拟实验现场的通信过程，此时需执行main.py，与正常执行流程相同
            若SEND_UDP_DIRECTLY=False则此时由程序本身调用Base并一次性完成全部数据的读取和处理，并模仿main.py发送融合结果

        3.启动可视化场景界面
            修改visualization.py/visualization3d.py中的场景参数
            在新cmd中执行python visualization.py / python visualization3d.py

        4.启动虚拟相机界面
            在新cmd中执行python visualizationCamera.py


##      可执行文件封装

        修改configs.py并使用Pyinstaller封装main.py为可执行程序即可。


#   测试

    测试使用一段已经记录的数据模拟一次实验过程，其数据为仿真时采集的记录，uav数为24，ugv数为8，使用原始UDP模式发送

    1.修改configs.py
        打开configs.py，修改下列参数：
        注释25行选择仿真场景并注释、解注释选择使用gazebo24对应参数
        设置SAVE_DATA为False，即不保存数据

    2.启动复现程序
        打开reproduce.py，修改下列参数：
        设置path为 r'(Path of fusion)\backup\datas\2022.07.29\\'，选择2022.07.29的数据文件夹
        设置tail为 "_14.24.47"，选择此后缀的记录
        设置SEND_UDP_DIRECTLY为True，选择直接发送UDP数据
        在新cmd中执行python reproduce.py

    3.启动场景可视化界面
        打开visualization3d.py，修改下列参数：
        注释、解注释场景 gazebo 24，使该场景参数生效
        在新cmd中执行python visualization3d.py

    4.启动虚拟相机界面
        在新cmd中执行python visualizationCamera.py

    5.启动融合程序
        在新cmd中执行python main.py


#   已知问题与待解决问题

##      已知问题

        该算法最大的逻辑问题在于：当一个匹配由于视野不完整造成了错误的匹配后，若长时间得不到其他修正信息，则融合结果会随该错误的匹配发生误差累积，如不同目标A、B出现在无人机视野的边缘处、且目标处于视野重叠区（该信息由UAV测量位姿获得），此时算法无法分辨两目标是同一目标或是两个目标由于观测误差存在而造成视野重叠的假象，此时若出现目标C并与上述融合结果发生融合，等同于在错误的融合结果上继续叠加错误的融合，造成最终结果的不可预测性。
        该问题有两个解决思路：一是定期对融合目标进行检查或抽查，即执行refusion过程，此时错误的匹配会由于目标分离而构成匹配不成立；二是考虑对视野边界增加缓冲阈值，即当目标进入缓冲边界阈值内再进行融合匹配。但从本质来看，本算法亟需一个合适的分离机制，当目标分离后应尽快实现重融合过程。

        此外，该算法中对最大匹配数以最小形状误差进行检验时，其过程是一个对可能结果穷举的NP问题，虽有进行剪枝操作，但在目标数较多且密集存在的最不利情况下，算法执行时间会显著上升，应考虑在复杂度较高的情况下使用一种智能算法（GA、PSO等）对该情况进行非精确化处理。

##      待解决问题

        算法中假设判断相邻关系的最大位置误差为误差等效正态分布的标准差的3倍，即3σ准则，由当前UAV状态（高低、速度、抖动等）决定，最初假设可由一个有限状态参数的经验公式获得（该方法预留在base.Target.getMaxPosError函数中），但由于从未实机测试，此最大位置误差在算法中被限定为一个固定的距离，且从未使用标准差σ，望以后实验中对该误差（即实际位置与测量位置的误差）进行统计，由UAV得出预估标准差σ并对最大位置误差进行修正。

