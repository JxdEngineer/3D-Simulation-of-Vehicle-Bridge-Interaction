function Fb=GetBridgeNodalF(xv,yv,node,dof_index,Fv)
% 此函数用于计算车辆荷载分配到桥梁结构上的节点荷载Fb
% xv是每一时刻汽车所在纵桥向x坐标
% yv是每一时刻汽车所在横桥向Y坐标
% node是ANSYS导入到MATLAB中的结点坐标
% dof_index是ANSYS导入到MATLAB中的节点坐标-自由度映射关系
% Fv是车轮荷载时间序列
% 下图是车轮所在桥梁结点网格示意图，用于距离反比法分配荷载
%%%%%%%%%%%%%%%%%%%%%%%%%%
%   4(x1,y2)  1(x2,y2)   %
%   3(x1,y1)  2(x2,y1)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% 同济大学桥梁系，简旭东，2020年12月06日

Z_deck=-0.1;
nodeb=node(node(:,3)==Z_deck,:);  %桥面节点的Z坐标为Z_deck
% 为什么这里不直接写node=node(node(:,3)==0,:)呢？因为这样会改变结点序号，导致下面的SearchDofIndex出错
Fb=zeros(length(dof_index),length(xv));
for i=1:length(xv)
    if i==250
        
    end
    if xv(i)>=min(nodeb(:,1)) && xv(i)<=max(nodeb(:,1)) && yv(i)>=min(nodeb(:,2)) && yv(i)<=max(nodeb(:,2)) %此时车轮轨迹在桥梁范围内
        temp=nodeb(nodeb(:,1)<=xv(i),1);temp=sort(temp); %排序便于取边界值
        x1=temp(end); %第i时刻车轮所在桥梁节点四边形网格的纵桥向坐标下界
        temp=nodeb(nodeb(:,1)>xv(i),1);temp=sort(temp); %排序便于取边界值
        if isempty(temp)   %temp为空时，车轮恰好作用在桥梁边界节点上，x1为桥梁边界节点纵坐标，x2为最靠近桥梁边界节点的纵坐标（注意这里有简化，没考虑CP节点。但是对结果应该影响不大，因为这只是最后一个荷载步）
            dof3=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
            dof4=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
            if isempty(dof3) %此时节点自由度被约束
                Fb(dof4,i)=Fv(i)*abs((yv(i)-y1)/(y2-y1));
            elseif isempty(dof4)
                Fb(dof3,i)=Fv(i)*abs((yv(i)-y2)/(y2-y1)); %此时轮位处位移使用一维距离反比法计算，只需考虑y即可
            else
                Fb(dof3,i)=Fv(i)*abs((yv(i)-y2)/(y2-y1)); %此时轮位处位移使用一维距离反比法计算，只需考虑y即可
                Fb(dof4,i)=Fv(i)*abs((yv(i)-y1)/(y2-y1));
            end
            continue  %跳出本次循环，直接进入下一次
        else
            x2=temp(1);    %第i时刻车轮所在桥梁节点四边形网格的纵桥向坐标上界，这里没有等号，避免节点重合情况出现
        end
        temp=nodeb(nodeb(:,2)<=yv(i),2);temp=sort(temp); %排序便于取边界值
        y1=temp(end); %第i时刻车轮所在桥梁节点四边形网格的横桥向坐标下界
        temp=nodeb(nodeb(:,2)>yv(i),2);temp=sort(temp); %排序便于取边界值
        y2=temp(1);    %第i时刻车轮所在桥梁节点四边形网格的横桥向坐标上界
        % 计算节点1分配到的节点荷载
        dof1=SearchDofIndex(x2,y2,Z_deck,node,dof_index,3);
        if ~isempty(dof1)  %dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，不用加力
            Fdof1=Fv(i)*abs((xv(i)-x1)/(x2-x1)*(yv(i)-y1)/(y2-y1)); %距离反比法计算结点力
            Fb(dof1,i)=Fdof1;
        end
        % 计算节点2分配到的节点荷载
        dof2=SearchDofIndex(x2,y1,Z_deck,node,dof_index,3);
        if ~isempty(dof2)  %dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，不用加力
            Fdof2=Fv(i)*abs((xv(i)-x1)/(x2-x1)*(yv(i)-y2)/(y2-y1)); %距离反比法计算结点力
            Fb(dof2,i)=Fdof2;
        end
        % 计算节点3分配到的节点荷载
        dof3=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
        if ~isempty(dof3)  %dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，不用加力
            Fdof3=Fv(i)*abs((xv(i)-x2)/(x2-x1)*(yv(i)-y2)/(y2-y1)); %距离反比法计算结点力
            Fb(dof3,i)=Fdof3;
        end
        % 计算节点4分配到的节点荷载
        dof4=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
        if ~isempty(dof4)  %dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，不用加力
            Fdof4=Fv(i)*abs((xv(i)-x2)/(x2-x1)*(yv(i)-y1)/(y2-y1)); %距离反比法计算结点力
            Fb(dof4,i)=Fdof4;
        end
    end
end