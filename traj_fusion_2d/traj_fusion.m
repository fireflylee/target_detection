%构造更新融合结果含参匿名函数，外部调用
updateFusion_fun=@(batch,fusion)(updateFusion(batch,fusion,kf_fun,dt2F2,H2,dt2Q2));

%更新融合结果
function fusion = updateFusion(batch, fusion, kf_fun, dt2F2, H2, dt2Q2)
    association=batch.association;
    if association~=0
        dt=(batch.t_last-fusion(association).t_last);
        if dt>0     %忽略dt<=0
            %构造R阵
            R2=eye(2)*500;
            %二次航迹滤波
            [fusion(association).x_kf, fusion(association).P_kf]=kf_fun(fusion(association).x_kf,fusion(association).P_kf,dt2F2(dt),H2,batch.traj(:,end),dt2Q2(dt),R2);
            fusion(association).traj=[fusion(association).traj,fusion(association).x_kf(1:2)];
            fusion(association).len=fusion(association).len+1;
            fusion(association).t_last=batch.t_last;
        end
    end
end