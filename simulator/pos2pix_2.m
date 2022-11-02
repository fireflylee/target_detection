function [pix] = pos2pix_2(pos,inmtx,uavPos,rotation)
%pos2pix_2 实际位置推算像素位置
%   pos=[X,Y,Z]:目标位于世界坐标系的位置,单位m
%   inmtx:相机内参矩阵
%   uavPos=[x,y,height]:uav位置,单位m
%   rotation=[yaw,pitch,roll]:摄像机旋转角,yaw->pitch->roll顺序,单位:°

%假设初始状态时光轴与世界坐标系y轴重合，相机坐标系x轴与世界坐标系x轴重合

resolution=[1920,1080];

yaw=rotation(1);
pitch=rotation(2);
roll=rotation(3);
yaw=yaw/180.0*pi;
pitch=pitch/180.0*pi;
roll=roll/180.0*pi;

%相机旋转矩阵（yaw->pitch->roll）并旋转到相机坐标系定义（z轴为光轴）
rotation_mat=[cos(-yaw),-sin(-yaw),0;sin(-yaw),cos(-yaw),0;0,0,1]*[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)]*[cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)]*[1,0,0;0,0,1;0,-1,0];
exmtx=inv([rotation_mat,uavPos;zeros(1,3),1]);

result=[inmtx,zeros(3,1)]*exmtx*[pos;1];
pix=round([result(1)/result(3),result(2)/result(3)]);

if pix(1)<0 || pix(1)>=resolution(1) || pix(2)<0 || pix(2)>=resolution(2)
    pix=[-1,-1];
end

end

