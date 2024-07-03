function Z=ModeDecomposition(Mn,Kn,zeta,P,Phi,Z0,dT,T)
% This function uses the modal decomposition method to calculate the dynamic response Z (real coordinates) of the structure
% M/K are the modal mass/stiffness of the structure respectively
% zeta is the modal damping ratio
% P is the structural load matrix in real coordinates
% Phi is the structural modal
% Z0 is the initial response in real coordinates, and the 1st/2nd/3rd columns are displacement, velocity, and acceleration respectively
% dT is the time step of dynamic calculation
% T is the total duration of dynamic calculation
% Tongji University Bridge Department, Jian Xudong, December 6, 2020

Cn=2*sqrt(Mn.*Kn)*zeta; 

modeN=length(Mn); %Number of vibration modes involved in the calculation
M=eye(modeN);
K=eye(modeN);
C=eye(modeN);
for i=1:modeN
    M(i,i)=Mn(i);  
    K(i,i)=Kn(i);  
    C(i,i)=Cn(i); 
end

Pn=Phi'*P; 
uq0=Phi'*Z0(:,1);  
vq0=Phi'*Z0(:,2); 
aq0=Phi'*Z0(:,3); 
Zq0=[uq0,vq0,aq0];
Zq=NewmarkBeta(K,M,C,Pn,Zq0,dT,T);
% Response restored to the real physical coordinate system
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
% Newmark-Constant Acceleration Method
% Z: Calculated node degree of freedom displacement/velocity/acceleration
% node: Node number
% element: Element node number
% K/M/C: Stiffness/mass/damping matrix
% P: Load vector
% Z0: Initial displacement matrix, the 1st/2nd/3rd column vectors are initial displacement/velocity/acceleration respectively
% deltaT: Time step
% T: Total calculation time
gamma=1/2;beta=1/4;
N=T/dT+1;
dof_n=length(M(:,1));
Z=cell(1,3);
Z{1}=zeros(dof_n,N); %displacement
Z{2}=zeros(dof_n,N); %velocity
Z{3}=zeros(dof_n,N); %acceleration
Z{1}(:,1)=Z0(:,1); %initial degree of freedom displacement
Z{2}(:,1)=Z0(:,2); %initial degree of freedom velocity
Z{3}(:,1)=Z0(:,3); %initial degree of freedom acceleration
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