function trz=GetTireDispRoad3D(rz,xv,yv,dx,dy)
% 此函数用于求每时刻车轮所在位置处的路面不平度
% 因为车轮位置不一定与生成的路面不平度坐标重合
% 所以采用线性插值法计算任意位置处的路面不平度
% 四点不能形成一个平面，故采用三角形插值法
% 具体细节参考《随机激励三维路面空间域模型建模与仿真》
% ts是每一时刻车轮底部的路面不平度
% rs是路面不平度，每一行表示X横桥向路面不平度剖面，每一列表示Y纵桥向路面不平度剖面
%     比如，rs(n,:)表示x坐标为dx*(n-1)的路面横向剖面
% xv是每一时刻汽车所在纵桥向x坐标
% yv是每一时刻汽车所在横桥向Y坐标
% dx是路面不平度纵桥向空间分辨率
% dy是路面不平度横桥向空间分辨率
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     D(x1,y2)  C(x2,y2)              %
%     A(x1,y1)  B(x2,y1)              %
%↑                                   %
%·→ 路面不平度坐标系原点在最左下角  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 同济大学桥梁系，简旭东，2020年7月7日

Nv=length(xv); %汽车轨迹向量长度
trz=zeros(1,Nv); %汽车轮迹处对应的路面不平度
Nx=length(rz(:,1)); %路面不平度剖面纵桥向采样点数
x_rs=(0:Nx-1)*dx; %路面不平度纵桥向坐标
Ny=length(rz(1,:)); %路面不平度剖面横桥向采样点数
y_rs=(0:Ny-1)*dy; %路面不平度横桥向坐标
for i=1:Nv
%     if i==1
%     aaaa=1
%     end
    temp=x_rs(x_rs<=xv(i));temp=sort(temp); %排序便于取边界值
    x1=temp(end); %第i时刻车轮所在路面不平度四边形网格的纵桥向坐标下界
    temp=x_rs(x_rs>xv(i));temp=sort(temp); %排序便于取边界值
    x2=temp(1); %第i时刻车轮所在路面不平度四边形网格的纵桥向坐标上界，这里没有等号，避免节点重合情况出现
    temp=y_rs(y_rs<=yv(i));temp=sort(temp); %排序便于取边界值
    y1=temp(end); %第i时刻车轮所在路面不平度四边形网格的横桥向坐标下界
    temp=y_rs(y_rs>yv(i));temp=sort(temp); %排序便于取边界值
    y2=temp(1); %第i时刻车轮所在路面不平度四边形网格的横桥向坐标上界
    i_x1=(x_rs==x1); %x1在路面不平度矩阵中的位置
    i_x2=(x_rs==x2); %x2在路面不平度矩阵中的位置
    i_y1=(y_rs==y1); %y1在路面不平度矩阵中的位置
    i_y2=(y_rs==y2); %y2在路面不平度矩阵中的位置
    tri1_x=[x1,x2,x2]; %三角形ABC三个角点的X坐标，三角形示意图见上
    tri1_y=[y1,y1,y2]; %三角形ABC三个角点的Y坐标
    tri2_x=[x1,x2,x1]; %三角形ACD三个角点的X坐标
    tri2_y=[y1,y2,y2]; %三角形ACD三个角点的Y坐标 
    if inpolygon(xv(i),yv(i),tri1_x,tri1_y) %此时车轮轨迹点P落在三角形ABC内
        A=abs(det([1,x1,y1;1,x2,y1;1,x2,y2])/2);  %三角形ABC的面积
        A1=abs(det([1,x1,y1;1,x2,y1;1,xv(i),yv(i)])/2); %三角形ABP的面积
        A2=abs(det([1,x2,y1;1,x2,y2;1,xv(i),yv(i)])/2); %三角形BCP的面积
        A3=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); %三角形CAP的面积
        trz(i)=rz(i_x2,i_y2)*A1/A+rz(i_x1,i_y1)*A2/A+rz(i_x2,i_y1)*A3/A; %三角形插值，参考《有限单元法》P106
    elseif inpolygon(xv(i),yv(i),tri2_x,tri2_y) %此时车轮轨迹点落在三角形ACD内
        A=abs(det([1,x1,y1;1,x2,y2;1,x1,y2])/2);  %三角形ACD的面积
        A1=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); %三角形ACP的面积
        A2=abs(det([1,x2,y2;1,x1,y2;1,xv(i),yv(i)])/2); %三角形CDP的面积
        A3=abs(det([1,x1,y2;1,x1,y1;1,xv(i),yv(i)])/2); %三角形DAP的面积
        trz(i)=rz(i_x1,i_y2)*A1/A+rz(i_x1,i_y1)*A2/A+rz(i_x2,i_y2)*A3/A; %三角形插值，参考《有限单元法》P106
    end
end
end