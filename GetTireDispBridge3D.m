function tbz=GetTireDispBridge3D(bz,xv,yv,node,dof_index)
% 此函数用于求每时刻车轮所在位置处的桥梁变形tbs
% 因为车轮位置不一定与生成的桥梁结点坐标重合
% 所以采用线性插值法计算任意位置处的桥梁变形
% 四点不能形成一个平面，故采用三角形插值法
% 具体细节参考《随机激励三维路面空间域模型建模与仿真》
% ts是每一时刻车轮底部的路面不平度
% bs是桥梁变形时间序列，每一列表示某时刻的桥梁变形
% xv是每一时刻汽车所在纵桥向x坐标
% yv是每一时刻汽车所在横桥向Y坐标
% node是桥梁结点坐标
% dof_index是桥梁结点-自由度映射关系
%%%%%%%%%%%%%%%%%%%%%%%%%%
%   D(x1,y2)  C(x2,y2)   %
%   A(x1,y1)  B(x2,y1)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% 同济大学桥梁系，简旭东，2020年12月06日

Z_deck=-0.1;
nodeb=node(node(:,3)==Z_deck,:);  %此行命令只使用node中Z坐标为Z_deck的点，用于寻找桥面板上的点
% 为什么这里不直接写node=node(node(:,3)==0,:)呢？因为这样会改变结点序号，导致下面的SearchDofIndex出错
tbz=zeros(1,length(xv));
for i=1:length(xv)
    if i==5625
        
    end
    if xv(i)>=min(nodeb(:,1)) && xv(i)<=max(nodeb(:,1)) && yv(i)>=min(nodeb(:,2)) && yv(i)<=max(nodeb(:,2)) %此时车轮轨迹在桥梁范围内
        temp=nodeb(nodeb(:,1)<=xv(i),1);temp=sort(temp); %排序便于取边界值
        x1=temp(end); %第i时刻车轮所在桥梁节点四边形网格的纵桥向坐标下界
        temp=nodeb(nodeb(:,1)>xv(i),1);temp=sort(temp); %排序便于取边界值
        if isempty(temp) %temp为空时，车轮恰好作用在桥梁边界节点上，x1为桥梁边界节点纵坐标，x2为最靠近桥梁边界节点的纵坐标（注意这里有简化，没考虑CP节点。但是对结果应该影响不大，因为这只是最后一个荷载步）
            dofA=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
            dofD=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
            if isempty(dofA)  %此时有边界约束，节点自由度不存在
                tbz(i)=(0*abs((yv(i)-y2)/(y1-y2))+bz(dofD,i)*abs((yv(i)-y1)/(y1-y2))); %此时轮位处位移使用一维距离反比法计算，只需考虑y即可
            elseif isempty(dofD)
                tbz(i)=(bz(dofA,i)*abs((yv(i)-y2)/(y1-y2))+0*abs((yv(i)-y1)/(y1-y2))); %此时轮位处位移使用一维距离反比法计算，只需考虑y即可
            else
                tbz(i)=(bz(dofA,i)*abs((yv(i)-y2)/(y1-y2))+bz(dofD,i)*abs((yv(i)-y1)/(y1-y2))); %此时轮位处位移使用一维距离反比法计算，只需考虑y即可
            end
            continue  %跳出本次循环，直接进入下一次
        else
            x2=temp(1);    %第i时刻车轮所在桥梁节点四边形网格的纵桥向坐标上界，这里没有等号，避免节点重合情况出现
        end
        temp=nodeb(nodeb(:,2)<=yv(i),2);temp=sort(temp); %排序便于取边界值
        y1=temp(end); %第i时刻车轮所在桥梁节点四边形网格的横桥向坐标下界
        temp=nodeb(nodeb(:,2)>yv(i),2);temp=sort(temp); %排序便于取边界值
        y2=temp(1);    %第i时刻车轮所在桥梁节点四边形网格的横桥向坐标上界
        %计算节点C对应的桥面竖向位移
        
        dofC=SearchDofIndex(x2,y2,Z_deck,node,dof_index,3);
        if isempty(dofC)  %dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，位移为0
            zC=0;
        else
            zC=bz(dofC,i);
        end
        
        %计算节点B对应的桥面竖向位移
        
        dofB=SearchDofIndex(x2,y1,Z_deck,node,dof_index,3);
        if isempty(dofB)  %CP集只有一个节点且dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，位移为0
            zB=0;
        else
            zB=bz(dofB,i);
        end
        
        %计算节点A对应的桥面竖向位移
        
        dofA=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
        if isempty(dofA)  %CP集只有一个节点且dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，位移为0
            zA=0;
        else
            zA=bz(dofA,i);
        end
        
        %计算节点D对应的桥面竖向位移
        dofD=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
        if isempty(dofD)  %CP集只有一个节点且dof为空集时该节点自由度不存在，即桥梁模型此处存在边界约束，位移为0
            zD=0;
        else
            zD=bz(dofD,i);
        end
        
        tri1_x=[x1,x2,x2]; %三角形ABC三个角点的X坐标，三角形示意图见上
        tri1_y=[y1,y1,y2]; %三角形ABC三个角点的Y坐标
        tri2_x=[x1,x2,x1]; %三角形ACD三个角点的X坐标
        tri2_y=[y1,y2,y2]; %三角形ACD三个角点的Y坐标
        if inpolygon(xv(i),yv(i),tri1_x,tri1_y) %此时车轮轨迹点P落在三角形ABC内
            A=abs(det([1,x1,y1;1,x2,y1;1,x2,y2])/2);  %三角形ABC的面积
            A1=abs(det([1,x1,y1;1,x2,y1;1,xv(i),yv(i)])/2); %三角形ABP的面积
            A2=abs(det([1,x2,y1;1,x2,y2;1,xv(i),yv(i)])/2); %三角形BCP的面积
            A3=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); %三角形CAP的面积
            tbz(i)=zC*A1/A+zA*A2/A+zB*A3/A; %三角形插值，参考《有限单元法》P106
        elseif inpolygon(xv(i),yv(i),tri2_x,tri2_y) %此时车轮轨迹点落在三角形ACD内
            A=abs(det([1,x1,y1;1,x2,y2;1,x1,y2])/2);  %三角形ACD的面积
            A1=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); %三角形ACP的面积
            A2=abs(det([1,x2,y2;1,x1,y2;1,xv(i),yv(i)])/2); %三角形CDP的面积
            A3=abs(det([1,x1,y2;1,x1,y1;1,xv(i),yv(i)])/2); %三角形DAP的面积
            tbz(i)=zD*A1/A+zA*A2/A+zC*A3/A; %三角形插值，参考《有限单元法》P106
        end
    end
end
end