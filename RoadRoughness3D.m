function h=RoadRoughness3D(Lx,Ly,dx,dy,class)
% this function uses the 2D IFFT to generate 3D isotropic road roughness surface
% h(x,y): road roughness surface
% Lx, Ly: length and width of the road roughness surface
% dx, dy: spatial resolution of the road roughness surface (default = 0.05 m)
% class: road roughness class£¬1-8 corresponds to A-H
% author: Jian Xudong, Tongji University
% reference: 
% Hu, Zhi Gang, et al. "Numerical Modeling and Simulation of Random Road Surface Using IFFT Method." Advanced Materials Research. Vol. 199. Trans Tech Publications Ltd, 2011.

alpha=0.0006;  % constant
rou=[0.0153,0.0306,0.0611,0.1222,0.2444,0.4888,0.9776,1.9552];  % road roughness class
rou=rou(class);
Nx=Lx/dx+1; 
Ny=Ly/dy+1;
a=4*pi*alpha;
fai=2*pi*rand(Nx,Ny);
F=zeros(Nx,Ny);
Gd=zeros(Nx,Ny);
for i=1:Nx
    for j=1:Ny
        fx=i/Nx/dx;
        fy=j/Ny/dy;
        Gd(i,j)=4*pi*a*rou^2/(a^2+4*pi^2*(fx^2+fy^2))^(3/2);  % PSD of the road roughness
        F(i,j)=sqrt(Nx*Ny/dx/dy*Gd(i,j))*exp(1j*fai(i,j));  % Fourier transform of the road roughness
    end
end
h=ifft2(F,'symmetric');   % 2D IFFT
end