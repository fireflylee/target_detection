%相关质量判断阈值
KR_max=3;
KN_max=2;
KT_max=3;
LEN_min=3;

%构造更新关联结果含参匿名函数，外部调用
updateAssociation_fun=@(batch,fusion)(updateAssociation(batch,fusion,updateRQ_fun,LEN_min,KR_max,KN_max,KT_max, rho_max));

%更新关联结果
function [batch,fusion] = updateAssociation(batch, fusion, updateRQ_fun, LEN_min, KR_max, KN_max, KT_max, rho_max)
    if batch.len>LEN_min                                    %忽略UKF滤波器介入后初始n点
        if size(fusion,1)==0                                %无二次航迹
            %二次航迹新批
            fusion=[fusion,struct('BATCH_NO',1,'traj',batch.traj(:,end),'len',1,'t_last',batch.t_last,'x_kf',batch.x_kf,'P_kf',eye(4)*10000)];
            batch.association=1;
        else
            batch=updateRQ_fun(batch,fusion);               %更新相关质量
            if batch.association==0                         %无关联
                if max(batch.RQ.KR)<KR_max                  %归一质量均不足阈值
                    if batch.RQ.KN<KN_max                   %离异质量不足阈值
                        return
                    else                                    %判断离异
                        %二次航迹新批
                        fusion=[fusion;struct('BATCH_NO',size(fusion,1)+1,'traj',batch.traj(:,end),'len',1,'t_last',batch.t_last,'x_kf',batch.x_kf,'P_kf',eye(4)*10000)];
                        batch.association=size(fusion,1);
                    end
                else                                        %判断合批
                    min_rho=rho_max;
                    association=0;
                    %寻找最匹配关联
                    for i=1:size(fusion,1)
                        if batch.RQ.KR(i)>=KR_max
                            if batch.RQ.rho(i)<min_rho
                                association=i;
                                min_rho=batch.RQ.rho(i);
                            end
                        end
                    end
                    batch.association=association;          %合批
                end
            else                                            %存在关联
                if batch.RQ.KT(batch.association)<KT_max    %判断分叉
                    %寻找最匹配关联
                    association=batch.association;
                    min_rho=batch.RQ.rho(association);
                    for i=1:size(fusion,1)
                        if batch.RQ.KR(i)>=KR_max
                            if batch.RQ.rho(i)<min_rho
                                association=i;
                                min_rho=batch.RQ.rho(i);
                            end
                        end
                    end
                    batch.association=association;      %变关联
                else
                    batch.association=0;                    %分叉，取消关联
                end
            end
        end
    end
end