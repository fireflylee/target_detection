%打印未关联点数

disp(['默认每批初始',num2str(LEN_min),'轨迹点被忽略不参与关联过程。'])
for i=1:size(stations,2)
    str=['站',num2str(i-1),'： '];
    sum=0;
    for j=1:size(stations(i).BATCHs,2)
        str=[str,num2str(stations(i).BATCHs(j).BATCH_NO),'批',num2str(stations(i).BATCHs(j).not_confirmed),'个, '];
        sum=sum+stations(i).BATCHs(j).not_confirmed;
    end
    str=[str,'总计',num2str(sum),'个。'];
    disp(str);
end