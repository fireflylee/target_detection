function [pos,isReal] = pix2pos_2(pix,inmtx,uavPos,rotation,d)
%pix2pos_2 像素位置推算实际位置
%   pix=[u,v]:目标位于图像中的位置,单位pixel
%   inmtx:相机内参矩阵
%   uavPos=[x,y,height]:uav位置,单位m
%   rotation=[yaw,pitch,roll]:摄像机旋转角,yaw->pitch->roll顺序,单位:°
%   d:相机光轴距地面直线距离,单位m

yaw=rotation(1);
pitch=rotation(2);
roll=rotation(3);
yaw=yaw/180.0*pi;
pitch=pitch/180.0*pi;
roll=roll/180.0*pi;

%相机旋转矩阵（yaw->pitch->roll）并旋转到相机坐标系定义（z轴为光轴）
rotation_mat=[cos(-yaw),-sin(-yaw),0;sin(-yaw),cos(-yaw),0;0,0,1]*[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)]*[cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)]*[1,0,0;0,0,1;0,-1,0];

T=-inv(rotation_mat)*uavPos;
inv_inmtx=inv(inmtx);

if d<0
    d=uavPos(3)/sin(-pitch);
end

%获得中心点水平面世界坐标系高度
center=rotation_mat*(inv_inmtx*d*[inmtx(1,3);inmtx(2,3);1]-T);
z=center(3);

%获得归一化平面处目标在世界坐标系中位置
posNorm=rotation_mat*(inv_inmtx*[pix(1);pix(2);1]-T);

pos=[uavPos(1)+(posNorm(1)-uavPos(1))/(uavPos(3)-posNorm(3))*(uavPos(3)-z);uavPos(2)+(posNorm(2)-uavPos(2))/(uavPos(3)-posNorm(3))*(uavPos(3)-z);z];
if posNorm(3)>uavPos(3)
    isReal=0;
else
    isReal=1;
end

end

