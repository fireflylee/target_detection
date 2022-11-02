clear 
close all;

%导入数据
% data = dlmread('indata.dat');
% Xs=data(:,1);
% Ys=data(:,2);
% STA_NOs=data(:,7)+1;
% BATCH_NOs=data(:,8);
% TIMESTAMPs=data(:,9);
fidin=fopen('data.txt','r');
count = 1;
while ~feof(fidin)
    tline=fgetl(fidin);
    data=jsondecode(tline);
    timestamp=data.timestamp;
    uav=data.uav;
    targets=data.targets;
    for i=1:size(targets)
        TIMESTAMPs(count,1)=timestamp;
        STA_NOs(count,1)=uav;
        BATCH_NOs(count,1)=targets(i).id;
        Xs(count,1)=targets(i).x;
        Ys(count,1)=targets(i).y;
        count=count+1;
    end
end
fclose(fidin);

for i=1:max(BATCH_NOs)*(max(STA_NOs))
    colors(i,:)=[rand,rand,rand];                   %批 配色
end
shape=['*','o','x','d'];
color_sta=['r','g','b','k'];
for i=1:max(STA_NOs)
    shapes(i)=shape(i);                             %站 点型
    colors_sta(i,:)=color_sta(i);                   %站 配色
    STA_BATCH_PAIR(i).STA=i;                        %站号-批号查找表
    STA_BATCH_PAIR(i).BATCHs=[];
    stations(i)=struct('STA_NO',i,'BATCHs',[]);     %站数据数组，包含各站各批轨迹及相应参数
end

%融合结果
fusion=[];

%各模块初始化
traj_ukf
traj_kf
traj_relevantQuality
traj_association
traj_fusion

%新建图形窗口
h = figure();                                                           % 创建图形窗口
warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');     % 关闭相关的警告提示
jFrame = get(h,'JavaFrame');                                            % 获取底层 Java 结构相关句柄
pause(0.1);                                                             % 不加停顿会报 Java 底层错误
set(jFrame,'Maximized',1);                                              % 设置其最大化为真
pause(0.1);                                                             % 如果不停顿，窗口可能来不及变化，所获取的窗口大小还是原来的尺寸
warning('on','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');		% 打开相关警告设置

subplot 131
title('一次航迹点')
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
axis equal
hold on

subplot 132
title('KF滤波一次航迹')
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
axis equal
hold on

subplot 133
title('二次航迹')
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
axis equal
hold on
pause(1)
