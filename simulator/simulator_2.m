clear 
close all;
colors=['r','g','b','k','r','g','b','k','r','g','b','k','r','g','b','k','r','g','b','k','r','g','b','k'];
resolution=[1920,1080];
FOV=[30,30/1920*1080];

mtx=[1/(tan(FOV(1)/2/180*pi)/(resolution(1)/2)),0,(resolution(1)/2);0,1/(tan(FOV(2)/2/180*pi)/(resolution(2)/2)),(resolution(2)/2);0,0,1];

%场地
FIELD=[500,500];
FIELD_LINES=[[0,FIELD(1),FIELD(1),0,0];[0,0,FIELD(2),FIELD(2),0]];

%视场
FOV=[30,15];
%uav数
UAV_NUM=5;
%目标数
TARGET_NUM=12;

%跟踪误差
UAV_POS_ERROR=10;
UAV_ROTATION_ERROR=2;
%测量误差
UAV_POS_ERROR2=10;
UAV_ROTATION_ERROR2=2;

%无人机初始位置、姿态、速度
uavs_pos=[FIELD(1)/UAV_NUM*[0:UAV_NUM-1]+FIELD(1)/2/UAV_NUM;ones(1,UAV_NUM)*(-700);ones(1,UAV_NUM)*200];
uavs_rotation=[zeros(1,UAV_NUM);ones(1,UAV_NUM)*(-25);zeros(1,UAV_NUM)];
uavs_speed=[zeros(1,UAV_NUM);ones(1,UAV_NUM)*(40);zeros(1,UAV_NUM)];

%目标初始位置、速度
targets_pos=[rand(1,TARGET_NUM)*FIELD(1)*0.4+FIELD(1)*0.3;rand(1,TARGET_NUM)*FIELD(2)*0.4+FIELD(2)*0.3;zeros(1,TARGET_NUM)];
targets_speed=[(rand(2,TARGET_NUM)-ones(2,TARGET_NUM)*0.5)*16;zeros(1,TARGET_NUM)];

latestIds=zeros(1,UAV_NUM);
isInFOVs=zeros(TARGET_NUM,UAV_NUM);
ids=zeros(TARGET_NUM,UAV_NUM);
pos2s=zeros(TARGET_NUM,UAV_NUM,3);

h = figure();                                                           % 创建图形窗口
warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');     % 关闭相关的警告提示
jFrame = get(h,'JavaFrame');                                            % 获取底层 Java 结构相关句柄
pause(0.1);                                                             % 不加停顿会报 Java 底层错误
set(jFrame,'Maximized',1);                                              % 设置其最大化为真
pause(0.1);                                                             % 如果不停顿，窗口可能来不及变化，所获取的窗口大小还是原来的尺寸
warning('on','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');		% 打开相关警告设置

%通信数据
fileID = fopen('data_2.txt','w');
fileID2 = fopen('result_2.txt','w');
fileID3 = fopen('real_2.txt','w');

%socket
t_client=udp('localhost','RemotePort',5000);
t_client.InputBufferSize=2048;
t_client.OutputBufferSize=2048;
fopen(t_client);
fprintf(t_client,'{"timestamp":-1}');   %复位
pause(1);
flushinput(t_client);
%fclose(t_client);

r_client=udp('localhost','LocalPort',33333);
r_client.EnablePortSharing='on';
r_client.InputBufferSize=2048;
fopen(r_client);
pause(0.1);
flushinput(r_client);
%fclose(r_client);

%current后缀为带跟踪误差的当前位置、姿态
uavs_pos_current=uavs_pos;
uavs_rotation_current=uavs_rotation;
%2后缀为带测量误差的位置、姿态、目标位置
uavs_pos2=uavs_pos;
uavs_rotation2=uavs_rotation;

dt=0.2;
for t=0:dt:FIELD(2)/40+10
    
    for i=1:UAV_NUM
        
        %步进
        uavs_pos=uavs_pos+uavs_speed*dt/UAV_NUM;
        targets_pos=targets_pos+targets_speed*dt/UAV_NUM;
        t_current=t+dt/UAV_NUM*(i-1);
        
        %添加跟踪误差
        uavs_pos_current(:,i)=uavs_pos(:,i)+(rand(3,1)-ones(3,1)*0.5)*UAV_POS_ERROR;
        uavs_rotation_current(:,i)=uavs_rotation(:,i)+(rand(3,1)-ones(3,1)*0.5)*UAV_ROTATION_ERROR;
        
        %添加测量误差
        uavs_pos2(:,i)=uavs_pos_current(:,i)+(rand(3,1)-ones(3,1)*0.5)*UAV_POS_ERROR2;
        uavs_rotation2(:,i)=uavs_rotation_current(:,i)+(rand(3,1)-ones(3,1))*UAV_ROTATION_ERROR2;
    
        clf

        subplot 131
        hold on
        axis equal

        xlim([-100 FIELD(1)+100])
        ylim([-800 FIELD(2)+100])

        plot(FIELD_LINES(1,:),FIELD_LINES(2,:),'b','linewidth',2);

        view(0,90);

        subplot 132
        hold on
        axis equal

        xlim([-100 FIELD(1)+100])
        ylim([-100 FIELD(2)+100])

        plot(FIELD_LINES(1,:),FIELD_LINES(2,:),'b','linewidth',2);

        subplot 133
        hold on
        axis equal

        xlim([-100 FIELD(1)+100])
        ylim([-100 FIELD(2)+100])

        plot(FIELD_LINES(1,:),FIELD_LINES(2,:),'b','linewidth',2);

        subplot 131
        for j=1:UAV_NUM
            drawFOVs2WithTarget(uavs_pos_current(:,j),uavs_rotation_current(:,j),FOV,targets_pos(1:2,:),colors(j))
        end
        
        targets_string='';
        d_current=uavs_pos_current(3,i)/sin(-uavs_rotation_current(2,i)/180*pi);
        d=d_current-2+4*rand;
        for j=1:TARGET_NUM
            pos=targets_pos(:,j);
            pix=pos2pix_2(pos,mtx,uavs_pos_current(:,i),uavs_rotation_current(:,i));
            if pix(1)<0
                if isInFOVs(j,i)==1
                    isInFOVs(j,i)=0;
                    ids(j,i)=0;
                end
            else
                if isInFOVs(j,i)==0
                    isInFOVs(j,i)=1;
                    latestIds(i)=latestIds(i)+1;
                    ids(j,i)=latestIds(i);
                end
                pos2=pix2pos_2(pix,mtx,uavs_pos2(:,i),uavs_rotation2(:,i),d);
                pos2s(j,i,:)=pos2;
                %disp(norm(pos2-pos,2)) %测量误差
                targets_string=[targets_string,'{"id":',num2str(ids(j,i)),',"x":',num2str(pos2(1)),',"y":',num2str(pos2(2)),',"z":',num2str(pos2(3)),'},'];
            end
        end
        if size(targets_string,2)>0
            targets_string=targets_string(1:size(targets_string,2)-1);
        end
        
        %位姿
        poseStr=replace(num2str([uavs_pos2(:,i)',uavs_rotation2(:,i)'],'%f,'),' ','');
        poseStr=['[',poseStr(1:end-1),']'];
        messageStr=['{"timestamp":',num2str(t_current*1000),',"uav":',num2str(i),',"pose":',poseStr,',"targets":[',targets_string,']}'];
        
        %disp(messageStr)
        
        %socket
        %t_client=tcpip('localhost',9999,'NetworkRole','client');
        %t_client.InputBufferSize=2048;
        %fopen(t_client);
        fprintf(t_client,messageStr);           %单无人机检测、测量结果
        
        fprintf(fileID,[messageStr,'\n']);
        
        realTargetsStr='[';
        for m=1:TARGET_NUM
            realTargetsStr=[realTargetsStr,'{"x":',num2str(targets_pos(1,m)),',"y":',num2str(targets_pos(2,m)),'},'];
        end
        realTargetsStr=[realTargetsStr(1:end-1),']'];
        fprintf(fileID3,[realTargetsStr,'\n']);
        
        subplot 132
        for m=1:UAV_NUM
            for n=1:TARGET_NUM
                if isInFOVs(n,m)
                    plot(pos2s(n,m,1),pos2s(n,m,2),[colors(m),'o']);
                end
            end
        end
        
        receiveStr=fscanf(r_client,'%s',2048);  %融合结果
        fprintf(fileID2,[receiveStr,'\n']);
        %fclose(t_client);
        %disp(receiveStr)
        fusion=jsondecode(receiveStr);
        subplot 133
        if size(fusion,1)>0
            for k=1:size(fusion.fusions,1)
                each=fusion.fusions(k);
                text(each.x+15,each.y+30,num2str(each.id))
                if strcmp(each.state,'Active')
                    plot(each.x,each.y,[colors(each.id),'o'],'MarkerFaceColor',colors(each.id));
                elseif strcmp(each.state,'Lost')
                    plot(each.x,each.y,[colors(each.id),'o']);
                elseif strcmp(each.state,'Nearly_Lost')
                    plot(each.x,each.y,[colors(each.id),'*']);
                end
            end
        end
        pause(0.01)
        %pause(dt/UAV_NUM)
    end
    %pause(0.01)
end
    
fclose(t_client);
fclose(fileID);
fclose(fileID2);
fclose(fileID3);