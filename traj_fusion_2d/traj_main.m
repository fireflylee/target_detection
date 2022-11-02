traj_init   %初始化

for i=1:size(Xs)
    %观测值坐标转换
    xy=[Xs(i);Ys(i)];
    
    %打印一次航迹点
    subplot 131
    plot(xy(1),xy(2),shapes(STA_NOs(i)),'color',colors((BATCH_NOs(i)-1)*(max(STA_NOs))+STA_NOs(i),:));
    
    if ~ismember(BATCH_NOs(i),STA_BATCH_PAIR(STA_NOs(i)).BATCHs)
        STA_BATCH_PAIR(STA_NOs(i)).BATCHs=[STA_BATCH_PAIR(STA_NOs(i)).BATCHs,BATCH_NOs(i)]; %拓展查找表
        %添加新批轨迹及参数，考虑到此时轨迹无速度信息故不参与二次航迹判定
        stations(STA_NOs(i)).BATCHs=[stations(STA_NOs(i)).BATCHs,...
            struct('BATCH_NO',BATCH_NOs(i),'traj',xy,'len',1,'t_last',TIMESTAMPs(i),'z',xy,'x_kf',zeros(4,1),'P_kf',eye(4)*10000,...
            'RQ',struct('KR',zeros(size(fusion,1),1),'KT',zeros(size(fusion,1),1),'KN',0,'rho',ones(size(fusion,1),1)*rho_max),'association',0,'not_confirmed',1)];
    else
        %提取记录对应批轨迹
        batch=stations(STA_NOs(i)).BATCHs(find(STA_BATCH_PAIR(STA_NOs(i)).BATCHs==BATCH_NOs(i)));
        batch.z=[Xs(i);Ys(i)];
        dt=(TIMESTAMPs(i)-batch.t_last);
        if batch.len==1
            %构造状态变量x
            batch.x_kf=[xy;(xy-batch.traj(1:2,end))/dt];
        else
            %KF滤波
            [batch.x_kf, batch.P_kf]=kf_fun(batch.x_kf, batch.P_kf, dt2F2(dt),H2, xy, dt2Q2(dt), eye(2)*500);
        end
        batch.traj=[batch.traj,batch.x_kf(1:2)];
        batch.len=batch.len+1;
        
        %打印滤波后一次航迹
        subplot 132
        plot(batch.traj(1,end-1:end),batch.traj(2,end-1:end),colors_sta(STA_NOs(i)),'linewidth',1);
        batch.t_last=TIMESTAMPs(i);
        
        %判断关联
        [batch,fusion]=updateAssociation_fun(batch,fusion);
        
        %融合
        fusion=updateFusion_fun(batch,fusion);
        
        %打印二次航迹
        if batch.association~=0
            if fusion(batch.association).len>=2 && batch.t_last>=fusion(batch.association).t_last   %取小于号时由于被fusion忽略故无更新
                subplot 133
                plot(fusion(batch.association).traj(1,end-1:end),fusion(batch.association).traj(2,end-1:end),'color',colors(batch.association,:),'linewidth',1);
            elseif fusion(batch.association).len==1
                subplot 133
                plot(fusion(batch.association).traj(1,end),fusion(batch.association).traj(2,end),'x','color',colors(batch.association,:));
            end
        else
            %记录未关联点
            batch.not_confirmed=batch.not_confirmed+1;
        end
        
        %更新stations
        stations(STA_NOs(i)).BATCHs(find(STA_BATCH_PAIR(STA_NOs(i)).BATCHs==BATCH_NOs(i)))=batch;
    end
    pause(0.002)
end

traj_plot           %打印
traj_statistics     %统计
