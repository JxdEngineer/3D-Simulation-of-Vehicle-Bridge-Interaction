%% Prepare road/bridge/vehicle model
clc
clear
close all
tic
dT=0.005;        
uv=10;  % m/s          
T=70/uv;         
%%%%%%%%%%%% road roughness model
RoadLx=200;    
RoadLy=200;   
Roaddx=0.05;  
Roaddy=0.05; 
RoadClass=1;  

rs=RoadRoughness3D(RoadLx,RoadLy,Roaddx,Roaddy,RoadClass);  % generate 3D road roughness surface
% load rs % load a existing road roughness surface
% rs=rs*0;  % ignore road roughness

xb_start=15;     
xr_start=-xb_start;    
RoadX=(0:Roaddx:RoadLx)+xr_start;
RoadY=(-RoadLy/2:Roaddy:RoadLy/2); 
[RoadX,RoadY]=meshgrid(RoadX,RoadY);
disp(['generating road roughness spends ',num2str(toc)])
%%%%%%%%%%%% bridge model
tic
% load bridge finite element model (created in ANSYS)
load Knb  % modal stiffness matrix 
load Mnb  % modal mass matrix 
load fb  % modal loads 
load Phib  % mode shape
load node % nodes of the model
load element % element of the mode
load dof_index % node-DOF mapping 
SpanL=20;  % length of the bridge span  
SpanN=2;   % number of the bridge span     
modeN=500; % number of modes used for the mode decomposition method
Knb=Knb(1:modeN);
Mnb=Mnb(1:modeN);
Phib=Phib(:,1:modeN);
% plot mode shape
% for i=1:6  
%     subplot(2,3,i)
%     node_mode=GetDeformation3D(Phib(:,i),node,dof_index,2000);
%     PlotModel(node_mode,element)
%     title(['frequency=',num2str(fb(i)),'Hz'])
% end
Zetan=0.02;  % damping ratio of each mode
disp(['loading bridge model spends ',num2str(toc)])
%%%%%%%%%%%% vehicle model
tic
% coordinate system of the vehicle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   wheel#3  wheel#1      
%                     → direction where the vehicle heads (X)
%   wheel#4  wheel#2  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sensing vehicle%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NumV=1;
load Mv % mass matrix of the 7-DOF full-car model
load Kv % stiffness matrix of the 7-DOF full-car model
load Cv % damping matrix of the 7-DOF full-car model

a1=1.5;
a2=1.5; 
b1=0.9; 
b2=0.9; 

coef_kt = 1;
mb=1040;
Ibx=1180;
Iby=1580;
m1=53;m2=53;
m3=76;m4=76;
vehicle(NumV).Gv(1)=(m1+mb/4)*9.807;  
vehicle(NumV).Gv(2)=(m2+mb/4)*9.807; 
vehicle(NumV).Gv(3)=(m3+mb/4)*9.807;  
vehicle(NumV).Gv(4)=(m4+mb/4)*9.807;  

c1=2000;
c2=2000;
c3=2000;
c4=2000;
ct1=430;
ct2=430;
ct3=430;
ct4=430;
vehicle(NumV).ct(1)=ct1;
vehicle(NumV).ct(2)=ct2;
vehicle(NumV).ct(3)=ct3;
vehicle(NumV).ct(4)=ct4;

kt1=200000*coef_kt;
kt2=200000*coef_kt;
kt3=200000*coef_kt;
kt4=200000*coef_kt;
vehicle(NumV).kt(1)=kt1;
vehicle(NumV).kt(2)=kt2;
vehicle(NumV).kt(3)=kt3;
vehicle(NumV).kt(4)=kt4;
k1=10000;
k2=10000;
k3=13000;
k4=13000;
kR=25000;

vehicle(NumV).Kv=eval(subs(Kv));
vehicle(NumV).Cv=eval(subs(Cv));
vehicle(NumV).Mv=eval(subs(Mv));

[~,fv]=eigs(vehicle(NumV).Kv,vehicle(NumV).Mv,length(vehicle(NumV).Kv),'sm');
vehicle(NumV).fv=sqrt(sort(diag(fv)))/2/pi;

xv_start=xr_start;    

vehicle(NumV).Ntire=4; 
vehicle(NumV).xv(4,:)=xv_start+uv*(0:dT:T); 
vehicle(NumV).yv(4,:)=zeros(1,T/dT+1)-5.5;   
vehicle(NumV).xv(3,:)=vehicle(NumV).xv(4,:);
vehicle(NumV).yv(3,:)=vehicle(NumV).yv(4,:)+b1+b2;
vehicle(NumV).xv(2,:)=vehicle(NumV).xv(4,:)+a1+a2;
vehicle(NumV).yv(2,:)=vehicle(NumV).yv(4,:);
vehicle(NumV).xv(1,:)=vehicle(NumV).xv(4,:)+a1+a2;
vehicle(NumV).yv(1,:)=vehicle(NumV).yv(4,:)+b1+b2;
% excitation vehicle%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NumV=2;
load Mv
load Kv
load Cv

a1=1.5*2; 
a2=1.5*2; 

b1=0.9*1.5;
b2=0.9*1.5;

amplify_factor = 20;
mb=1040*amplify_factor;
Ibx=1180*amplify_factor;
Iby=1580*amplify_factor;
m1=53*amplify_factor;m2=53*amplify_factor;
m3=76*amplify_factor;m4=76*amplify_factor;
vehicle(NumV).Gv(1)=(m1+mb/4)*9.807;  
vehicle(NumV).Gv(2)=(m2+mb/4)*9.807;  
vehicle(NumV).Gv(3)=(m3+mb/4)*9.807;  
vehicle(NumV).Gv(4)=(m4+mb/4)*9.807;  

c1=2000*amplify_factor;
c2=2000*amplify_factor;
c3=2000*amplify_factor;
c4=2000*amplify_factor;
ct1=430*amplify_factor;
ct2=430*amplify_factor;
ct3=430*amplify_factor;
ct4=430*amplify_factor;
vehicle(NumV).ct(1)=ct1;
vehicle(NumV).ct(2)=ct2;
vehicle(NumV).ct(3)=ct3;
vehicle(NumV).ct(4)=ct4;

kt1=200000*amplify_factor;
kt2=200000*amplify_factor;
kt3=200000*amplify_factor;
kt4=200000*amplify_factor;
vehicle(NumV).kt(1)=kt1;
vehicle(NumV).kt(2)=kt2;
vehicle(NumV).kt(3)=kt3;
vehicle(NumV).kt(4)=kt4;
k1=10000*amplify_factor;
k2=10000*amplify_factor;
k3=13000*amplify_factor;
k4=13000*amplify_factor;
kR=25000*amplify_factor;

vehicle(NumV).Kv=eval(subs(Kv));
vehicle(NumV).Cv=eval(subs(Cv));
vehicle(NumV).Mv=eval(subs(Mv));

[~,fv]=eigs(vehicle(NumV).Kv,vehicle(NumV).Mv,length(vehicle(NumV).Kv),'sm');
vehicle(NumV).fv=sqrt(sort(diag(fv)))/2/pi;

xv_start=xr_start;     
% uv=2;     % speed (m/s)
vehicle(NumV).Ntire=4;

vehicle(NumV).xv(4,:)=xv_start+uv*(0:dT:T); 
vehicle(NumV).yv(4,:)=zeros(1,T/dT+1)-2;  

vehicle(NumV).xv(3,:)=vehicle(NumV).xv(4,:);
vehicle(NumV).yv(3,:)=vehicle(NumV).yv(4,:)+b1+b2;

vehicle(NumV).xv(2,:)=vehicle(NumV).xv(4,:)+a1+a2;
vehicle(NumV).yv(2,:)=vehicle(NumV).yv(4,:);

vehicle(NumV).xv(1,:)=vehicle(NumV).xv(4,:)+a1+a2;
vehicle(NumV).yv(1,:)=vehicle(NumV).yv(4,:)+b1+b2;

disp(['generating vehicle models spends ',num2str(toc)])
Nv=length(vehicle);  
%% Calculating the dynamic response
tic
% initial condition
for k=1:Nv
    vehicle(k).Zv0=zeros(length(vehicle(k).Kv),3);
end
Zb0=zeros(length(dof_index),3);

for k=1:Nv
    vehicle(k).Fvr=zeros(length(vehicle(k).Kv),T/dT+1); 
    for kk=1:vehicle(k).Ntire 
        vehicle(k).tzr(kk,:)=GetTireDispRoad3D(rs,vehicle(k).xv(kk,:)-min(vehicle(k).xv(:,1)),vehicle(k).yv(kk,:)+RoadLy/2,Roaddx,Roaddy);
        vehicle(k).Fvr(kk,:)=vehicle(k).tzr(kk,:)*vehicle(k).kt(kk)+vehicle(k).ct(kk)*[0,diff(vehicle(k).tzr(kk,:))/dT]; 
    end
    vehicle(k).Zv=NewmarkBeta(vehicle(k).Kv,vehicle(k).Mv,vehicle(k).Cv,vehicle(k).Fvr,vehicle(k).Zv0,dT,T); 
end

Fb=cell(1,1);
Fb{1}=zeros(length(dof_index),T/dT+1);
for k=1:Nv
    for kk=1:vehicle(k).Ntire
        Fbr=GetBridgeNodalF(vehicle(k).xv(kk,:),vehicle(k).yv(kk,:),...
            node,dof_index,...
            vehicle(k).kt(kk)*(vehicle(k).Zv{1}(kk,:)-vehicle(k).tzr(kk,:))+vehicle(k).ct(kk)*(vehicle(k).Zv{2}(kk,:)-[0,diff(vehicle(k).tzr(kk,:))/dT])-vehicle(k).Gv(kk));
        Fb{1}=Fb{1}+Fbr;
    end
end

% start calculating
count=1;
while 1
    Zb=ModeDecomposition(Mnb,Knb,Zetan,Fb{count},Phib,Zb0,dT,T);  % updating bridge response
    for k=1:Nv
        vehicle(k).Fvrb=zeros(length(vehicle(k).Kv),T/dT+1); 
        for kk=1:vehicle(k).Ntire 
            vehicle(k).tzb(kk,:)=GetTireDispBridge3D(Zb{1},vehicle(k).xv(kk,:),vehicle(k).yv(kk,:),node,dof_index);
            vehicle(k).Fvrb(kk,:)=vehicle(k).kt(kk)*(vehicle(k).tzr(kk,:)+vehicle(k).tzb(kk,:))+...
                vehicle(k).ct(kk)*([0,diff(vehicle(k).tzr(kk,:))/dT]+[0,diff(vehicle(k).tzb(kk,:))/dT]); 
        end
        vehicle(k).Zv=NewmarkBeta(vehicle(k).Kv,vehicle(k).Mv,vehicle(k).Cv,vehicle(k).Fvrb,vehicle(k).Zv0,dT,T); % updating the dynamic response of vehicles
    end
    Fb{count+1}=zeros(length(dof_index),T/dT+1); 
    for k=1:Nv
        for kk=1:vehicle(k).Ntire  
            Fbrb=GetBridgeNodalF(vehicle(k).xv(kk,:),vehicle(k).yv(kk,:),...
                node,dof_index,...
                vehicle(k).kt(kk)*(+vehicle(k).Zv{1}(kk,:)-vehicle(k).tzr(kk,:)-vehicle(k).tzb(kk,:))+...
                vehicle(k).ct(kk)*(+vehicle(k).Zv{2}(kk,:)-[0,diff(vehicle(k).tzr(kk,:))/dT]-[0,diff(vehicle(k).tzb(kk,:))/dT])-vehicle(k).Gv(kk));
            Fb{count+1}=Fb{count+1}+Fbrb;
        end
    end
    error(count)=max(max(abs(Fb{count+1}-Fb{count}))); 
    if error(count)<0.01  % stop if reach convergence
        break
    end
    fprintf('VBI iteration round: %d, convergence error: %d\n',count, error)
    count=count+1;
end
disp(['VBI solving time (sec): ',num2str(toc)])
% save results
tic

% save output % save all variables

dof = SearchDofIndex(10.102041000000000,-6,-0.1,node,dof_index,3);  % bridge acc.
signal_b = Zb{3}(dof,:);
clear Zb Phib rs RoadX RoadY Zb0 Fbr Fb dof_index Fbrb
% save output.mat  

disp(['Results saving time (sec): ',num2str(toc)])