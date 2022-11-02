%模型参数
%由dt获得F2阵
dt2F2=@(dt)[1,0,dt,0;
            0,1,0,dt;
            0,0,1,0;
            0,0,0,1];
H2=[1,0,0,0;
    0,1,0,0];
%由dt获得Q2阵
dt2Q2=@(dt)[dt^4/4,0,dt^3/2,0;
           0,dt^4/4,0,dt^3/2;
           dt^3/2,0,dt^2,0;
           0,dt^3/2,0,dt^2];

%构造KF滤波含参匿名函数，外部调用
kf_fun=@(X,P,F,H,z,Q,R)(kf(X,P,F,H,z,Q,R));

%标准KF滤波
function [X,P]=kf(X,P,F,H,z,Q,R)
    X1=F*X;
    P1=F*P*F'+Q;
    K=P1*H'/(H*P1*H'+R);
    X=X1+K*(z-H*X1);
    P=P1-K*H*P1;
end