function trz=GetTireDispRoad3D(rz,xv,yv,dx,dy)
% This function is used to calculate the road surface roughness at the wheel position at each moment
% Because the wheel position does not necessarily coincide with the generated road surface roughness coordinates
% So linear interpolation is used to calculate the road surface roughness at any position
% Four points cannot form a plane, so triangular interpolation is used
% For specific details, please refer to "Modeling and Simulation of Random Excitation Three-Dimensional Road Surface Spatial Domain Model"
% ts is the road surface roughness at the bottom of the wheel at each moment
% rs is the road surface roughness, each row represents the road surface roughness profile in the X-transverse bridge direction, and each column represents the road surface roughness profile in the Y-longitudinal bridge direction
% For example, rs(n,:) represents the transverse road surface profile with an x-coordinate of dx*(n-1)
% xv is the x-coordinate in the longitudinal bridge direction of the car at each moment
% yv is the Y-coordinate in the transverse bridge direction of the car at each moment
% dx is the spatial resolution of the road surface roughness in the longitudinal bridge direction
% dy is the spatial resolution of the road surface roughness in the transverse bridge direction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     D(x1,y2)  C(x2,y2)              %
%     A(x1,y1)  B(x2,y1)              %
%¡ü                                   %
%¡¤¡ú The origin of the road roughness coordinate system is at the lower left corner  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tongji University Bridge Department, Jian Xudong, July 7, 2020

Nv=length(xv); %Car track vector length
trz=zeros(1,Nv); %Road surface roughness corresponding to the car wheel track
Nx=length(rz(:,1)); %Road surface roughness profile longitudinal bridge direction sampling points
x_rs=(0:Nx-1)*dx; %Road surface roughness longitudinal bridge direction coordinates
Ny=length(rz(1,:)); %Road surface roughness profile transverse bridge direction sampling points
y_rs=(0:Ny-1)*dy; %Road surface roughness transverse bridge direction coordinates
for i=1:Nv
%     if i==1
%     aaaa=1
%     end
    temp=x_rs(x_rs<=xv(i));temp=sort(temp);
    x1=temp(end);
    temp=x_rs(x_rs>xv(i));temp=sort(temp); 
    x2=temp(1); 
    temp=y_rs(y_rs<=yv(i));temp=sort(temp);
    y1=temp(end); 
    temp=y_rs(y_rs>yv(i));temp=sort(temp);
    y2=temp(1); 
    i_x1=(x_rs==x1);
    i_x2=(x_rs==x2);
    i_y1=(y_rs==y1); 
    i_y2=(y_rs==y2); 
    tri1_x=[x1,x2,x2]; 
    tri1_y=[y1,y1,y2]; 
    tri2_x=[x1,x2,x1]; 
    tri2_y=[y1,y2,y2]; 
    if inpolygon(xv(i),yv(i),tri1_x,tri1_y) 
        A=abs(det([1,x1,y1;1,x2,y1;1,x2,y2])/2);  
        A1=abs(det([1,x1,y1;1,x2,y1;1,xv(i),yv(i)])/2); 
        A2=abs(det([1,x2,y1;1,x2,y2;1,xv(i),yv(i)])/2); 
        A3=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); 
        trz(i)=rz(i_x2,i_y2)*A1/A+rz(i_x1,i_y1)*A2/A+rz(i_x2,i_y1)*A3/A; 
    elseif inpolygon(xv(i),yv(i),tri2_x,tri2_y) 
        A=abs(det([1,x1,y1;1,x2,y2;1,x1,y2])/2);  
        A1=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); 
        A2=abs(det([1,x2,y2;1,x1,y2;1,xv(i),yv(i)])/2); 
        A3=abs(det([1,x1,y2;1,x1,y1;1,xv(i),yv(i)])/2); 
        trz(i)=rz(i_x1,i_y2)*A1/A+rz(i_x1,i_y1)*A2/A+rz(i_x2,i_y2)*A3/A; 
    end
end
end