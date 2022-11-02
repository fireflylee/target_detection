rho_max=1;   %关联检验阈值

%构造更新相关质量含参匿名函数，外部调用
updateRQ_fun=@(batch,fusion)(updateRQ(batch,fusion,rho_max,dt2F2,dt2Q2));

%更新相关质量
function batch = updateRQ(batch, fusion, rho_max, dt2F2, dt2Q2)
    x_first=batch.x_kf;             %某批新数据滤波后状态
    P_first=batch.P_kf;             %协方差
    t_first_now=batch.t_last;       %数据到达时间
    RQ=batch.RQ;
    if size(RQ.KR,1)<size(fusion,1) %扩展维数至所有二次航迹
        rows=size(RQ.KR,1);
        RQ.KR=[RQ.KR;zeros(size(fusion,1)-rows,1)];
        RQ.KT=[RQ.KT;zeros(size(fusion,1)-rows,1)];
        RQ.rho=[RQ.rho;ones(size(fusion,1)-rows,1)*rho_max];
    end
    total=0;                        %关联数，用于判断离异质量
    for i=1:size(fusion,1)
        rho=getRho(x_first,P_first,fusion(i).x_kf,fusion(i).P_kf,t_first_now,fusion(i).t_last,dt2F2,dt2Q2);
        RQ.rho(i)=rho;
        if rho<rho_max              %一次航迹与二次航迹i关联
            RQ.KR(i)=RQ.KR(i)+1;    %归一质量增加
            RQ.KT(i)=0;             %脱离质量清空
            total=total+1;          %增加关联数
        else                        %非关联
          	RQ.KR(i)=0;             %归一质量清空
            RQ.KT(i)=RQ.KT(i)+1;    %脱离质量增加
        end
    end
    if total==0                     %全不关联
        RQ.KN=RQ.KN+1;              %离异质量增加
    else
        RQ.KN=0;                    %离异质量清空
    end
    batch.RQ=RQ;                    %更新相关质量
end

%相似性充分统计量
function rho = getRho(x_first, P_first, x_second, P_second, t_first_now, t_second_last, dt2F2, dt2Q2)
    if t_first_now == t_second_last %无需对二次航迹进行预测
        t = x_first - x_second;
        rho = t'*((P_first+P_second)^-1)*t;
    else
        dt=(t_first_now-t_second_last);
        if dt>0
            %预测二次航迹至当前时间
            x_second1=dt2F2(dt)*x_second;
            P_second1=dt2F2(dt)*P_second*dt2F2(dt)'+dt2Q2(dt);
            t = x_first - x_second1;
            rho = t'*((P_first+P_second1)^-1)*t;
        else
            %预测一次航迹至当前时间，直接使用二次航迹模型更新
            x_first1=dt2F2(dt)*x_first;
            P_first1=dt2F2(dt)*P_first*dt2F2(dt)'+dt2Q2(dt);
            t = x_first1 - x_second;
            rho = t'*((P_first1+P_second)^-1)*t;
        end
    end
end