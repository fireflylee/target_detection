%打印时均忽略z轴
figure()
hold on
title('一次航迹点')
Hs=[];
Labels=[];
indices=[];
for i=1:size(stations,2)
    indices=[indices,find(STA_NOs==i,1,'first')];
    Labels=[Labels;strcat('站',num2str(i))];
end
for i=1:size(Xs)
    Hs(i)=plot(Xs(i),Ys(i),shapes(STA_NOs(i)),'color',colors((BATCH_NOs(i)-1)*(max(STA_NOs))+STA_NOs(i),:));
end
legend(Hs(indices),Labels);
xlabel('x/m')
ylabel('y/m')
axis equal

figure()
hold on
title('KF滤波一次航迹')
Hs=[];
Labels=[];
for i=1:size(stations,2)
    for j=1:size(stations(i).BATCHs,2)
        H(i,j)=plot(stations(i).BATCHs(j).traj(1,:),stations(i).BATCHs(j).traj(2,:),colors_sta(i),'linewidth',1);
    end
    %Hs=[Hs,H(i,1)];
    %Labels=[Labels;strcat('站',num2str(i))];
end
%legend(Hs,Labels);
xlabel('x/m')
ylabel('y/m')
axis equal

figure()
hold on
title('二次航迹')
Hs=[];
Labels=[];
for i=1:size(fusion)
    H1=plot(fusion(i).traj(1,:),fusion(i).traj(2,:),'color',colors(i,:),'linewidth',1);
    H2=plot(fusion(i).traj(1,1),fusion(i).traj(2,1),'x','color',colors(i,:));
    H3=plot(fusion(i).traj(1,end),fusion(i).traj(2,end),'o','color',colors(i,:));
    Hs=[Hs,H1,H2,H3];
    Labels=[Labels;strcat('航迹',num2str(i));strcat('航迹',num2str(i),"起点");strcat('航迹',num2str(i),'终点')];
end
legend(Hs,Labels);
xlabel('x/m')
ylabel('y/m')
axis equal