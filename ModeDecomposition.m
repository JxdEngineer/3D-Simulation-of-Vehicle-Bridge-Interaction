function Z=ModeDecomposition(Mn,Kn,zeta,P,Phi,Z0,dT,T)
% 此函数使用振型分解法计算结构的动力响应Z（真实坐标）
% M/K分别是结构的振型质量/刚度
% zeta是振型阻尼比
% P是真实坐标下的结构荷载矩阵
% Phi是结构振型
% Z0是真实坐标下的初始响应，第1/2/3列分别为位移、速度、加速度
% dT动力计算时间步长
% T动力计算总时长
% 同济大学桥梁系，简旭东，2020年12月06日

Cn=2*sqrt(Mn.*Kn)*zeta; %振型阻尼

modeN=length(Mn); %参与计算的振型阶数
M=eye(modeN);
K=eye(modeN);
C=eye(modeN);
for i=1:modeN
    M(i,i)=Mn(i);  %振型质量对角矩阵
    K(i,i)=Kn(i);  %振型刚度对角矩阵
    C(i,i)=Cn(i);  %振型阻尼对角矩阵
end

Pn=Phi'*P; %振型荷载
uq0=Phi'*Z0(:,1);  %振型初位移
vq0=Phi'*Z0(:,2); %振型初速度
aq0=Phi'*Z0(:,3); %振型初加速度
Zq0=[uq0,vq0,aq0];
Zq=NewmarkBeta(K,M,C,Pn,Zq0,dT,T);
% 响应还原到真实物理坐标系
dofN=length(Phi(:,1));
N=T/dT+1;
u=zeros(dofN,N);
v=zeros(dofN,N);
a=zeros(dofN,N);
for i=1:modeN
    u=u+Phi(:,i).*Zq{1}(i,:);
    v=v+Phi(:,i).*Zq{2}(i,:);
    a=a+Phi(:,i).*Zq{3}(i,:);
end
Z=cell(1,3);
Z{1}=u;
Z{2}=v;
Z{3}=a;
end

function Z=NewmarkBeta(K,M,C,P,Z0,dT,T)
% Newmark-常加速度法
% Z：计算得到的节点自由度位移/速度/加速度
% node：节点编号
% element：单元节点号
% K/M/C：刚度/质量/阻尼矩阵
% P：荷载向量
% Z0：初始位移矩阵，第1/2/3列向量依次是初始位移/速度/加速度
% deltaT：时间步长
% T：总计算时间
gamma=1/2;beta=1/4;
N=T/dT+1;
dof_n=length(M(:,1));
Z=cell(1,3);
Z{1}=zeros(dof_n,N);   %位移
Z{2}=zeros(dof_n,N);   %速度
Z{3}=zeros(dof_n,N);   %加速度
Z{1}(:,1)=Z0(:,1);   %初始自由度位移
Z{2}(:,1)=Z0(:,2);  %初始自由度速度
Z{3}(:,1)=Z0(:,3);  %初始自由度加速度
a0=1/beta/dT^2;
a1=gamma/beta/dT;
a2=1/beta/dT;
a3=1/2/beta-1;
a4=gamma/beta-1;
a5=dT/2*(gamma/beta-2);
a6=dT*(1-gamma);
a7=gamma*dT;
K_eq=K+a0*M+a1*C;
for i=1:N-1
    P_eq(:,i+1)=P(:,i+1)+M*(a0*Z{1}(:,i)+a2*Z{2}(:,i)+...
        a3*Z{3}(:,i))+C*(a1*Z{1}(:,i)+a4*Z{2}(:,i)+a5*Z{3}(:,i));
    Z{1}(:,i+1)=K_eq\P_eq(:,i+1);
    Z{3}(:,i+1)=a0*(Z{1}(:,i+1)-Z{1}(:,i))-a2*Z{2}(:,i)-a3*Z{3}(:,i);
    Z{2}(:,i+1)=Z{2}(:,i)+a6*Z{3}(:,i)+a7*Z{3}(:,i+1);
end
end