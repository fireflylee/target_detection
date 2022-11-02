%模型参数
dt2f_fun=@(dt)(@(x)[x(1)+x(4)*dt;x(2)+x(5)*dt;x(3)+x(6)*dt;x(4);x(5);x(6)]);                                            %由dt获得状态方程，[x;y;z;vx;vy;vz]
h_fun=@(x)[sqrt(x(1)^2+x(2)^2+x(3)^2); mod(atan2(x(2),x(1)),2*pi)/pi*3000; atan2(x(3),sqrt(x(1)^2+x(2)^2))/pi*3000];    %观测方程，[距离;方位角;高低角]
%由dt获得Q阵
dt2Q=@(dt)[dt^4/4,0,0,dt^3/2,0,0;
           0,dt^4/4,0,0,dt^3/2,0;
           0,0,dt^4/4,0,0,dt^3/2;
           dt^3/2,0,0,dt^2,0,0;
           0,dt^3/2,0,0,dt^2,0;
           0,0,dt^3/2,0,0,dt^2];
R=diag([100^2,(0.5/360*6000)^2,(0.5/360*6000)^2]);    %R阵

%UKF参数初始化
L=6;            %状态维数
m=3;            %观测维数
alpha=1e-2;
ki=0;
beta=2;
lambda=alpha^2*(L+ki)-L;
c=L+lambda;
Wm=[lambda/c 0.5/c+zeros(1,2*L)];
Wc=Wm;
Wc(1)=Wc(1)+(1-alpha^2+beta);
c=sqrt(c);

%构造UKF滤波含参匿名函数，外部调用
ukf_fun=@(f_fun,X,P,h_fun,Z,Q,R)(ukf(f_fun,X,P,h_fun,Z,Q,R,L,m,c,Wm,Wc));

%UKF滤波
function [X, P]=ukf(f_fun, X, P, h_fun, Z, Q, R, L, m, c, Wm, Wc)
    Xsigmaset=sigmas(X, P, c);                          %获取sigma点集
    [X1means,X1,P1,X2]=ut(f_fun,Xsigmaset,Wm,Wc,L,Q);   %对sigma点集一步预测，对状态UT变换
    [Zpre,~,Pzz,Z2]=ut(h_fun,X1,Wm,Wc,m,R);             %得到观测预测
    Pxz=X2*diag(Wc)*Z2';                                %协方差Pxz
    K=Pxz/Pzz;                                          %计算Kalman增益
    deltaZ=Z-Zpre;
    while deltaZ(2)<-3000
        deltaZ(2)=deltaZ(2)+6000;
    end
    while deltaZ(2)>3000
        deltaZ(2)=deltaZ(2)-6000;
    end                                                 %修正方位角0密位附近的跳跃
    X=X1means+K*deltaZ;                                 %状态更新
    P=P1-K*Pxz';                                        %协方差更新
end

%UT变换
function [Xmeans, Xsigma_pre, P, Xdiv] = ut(fun, Xsigma, Wm, Wc, n, COV)
    LL=size(Xsigma,2);
    Xmeans=zeros(n,1);
    Xsigma_pre=zeros(n,LL);
    for k=1:LL
        Xsigma_pre(:,k)=fun(Xsigma(:,k));
        Xmeans=Xmeans+Wm(k)*Xsigma_pre(:,k);
    end
    Xdiv=Xsigma_pre-Xmeans(:,ones(1,LL));
    P=Xdiv*diag(Wc)*Xdiv'+COV;
end

%产生sigma点集
function Xset = sigmas(X, P, c)
    A=c*chol(P)';
    Y=X(:,ones(1,numel(X)));
    Xset=[X Y+A Y-A];
end