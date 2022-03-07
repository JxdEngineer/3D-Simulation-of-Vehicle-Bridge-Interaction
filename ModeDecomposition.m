function Z=ModeDecomposition(Mn,Kn,zeta,P,Phi,Z0,dT,T)
% �˺���ʹ�����ͷֽⷨ����ṹ�Ķ�����ӦZ����ʵ���꣩
% M/K�ֱ��ǽṹ����������/�ն�
% zeta�����������
% P����ʵ�����µĽṹ���ؾ���
% Phi�ǽṹ����
% Z0����ʵ�����µĳ�ʼ��Ӧ����1/2/3�зֱ�Ϊλ�ơ��ٶȡ����ٶ�
% dT��������ʱ�䲽��
% T����������ʱ��
% ͬ�ô�ѧ����ϵ�����񶫣�2020��12��06��

Cn=2*sqrt(Mn.*Kn)*zeta; %��������

modeN=length(Mn); %�����������ͽ���
M=eye(modeN);
K=eye(modeN);
C=eye(modeN);
for i=1:modeN
    M(i,i)=Mn(i);  %���������ԽǾ���
    K(i,i)=Kn(i);  %���͸նȶԽǾ���
    C(i,i)=Cn(i);  %��������ԽǾ���
end

Pn=Phi'*P; %���ͺ���
uq0=Phi'*Z0(:,1);  %���ͳ�λ��
vq0=Phi'*Z0(:,2); %���ͳ��ٶ�
aq0=Phi'*Z0(:,3); %���ͳ����ٶ�
Zq0=[uq0,vq0,aq0];
Zq=NewmarkBeta(K,M,C,Pn,Zq0,dT,T);
% ��Ӧ��ԭ����ʵ��������ϵ
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
% Newmark-�����ٶȷ�
% Z������õ��Ľڵ����ɶ�λ��/�ٶ�/���ٶ�
% node���ڵ���
% element����Ԫ�ڵ��
% K/M/C���ն�/����/�������
% P����������
% Z0����ʼλ�ƾ��󣬵�1/2/3�����������ǳ�ʼλ��/�ٶ�/���ٶ�
% deltaT��ʱ�䲽��
% T���ܼ���ʱ��
gamma=1/2;beta=1/4;
N=T/dT+1;
dof_n=length(M(:,1));
Z=cell(1,3);
Z{1}=zeros(dof_n,N);   %λ��
Z{2}=zeros(dof_n,N);   %�ٶ�
Z{3}=zeros(dof_n,N);   %���ٶ�
Z{1}(:,1)=Z0(:,1);   %��ʼ���ɶ�λ��
Z{2}(:,1)=Z0(:,2);  %��ʼ���ɶ��ٶ�
Z{3}(:,1)=Z0(:,3);  %��ʼ���ɶȼ��ٶ�
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