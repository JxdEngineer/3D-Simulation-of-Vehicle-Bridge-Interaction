function tbz=GetTireDispBridge3D(bz,xv,yv,node,dof_index)
% This function is used to calculate the bridge deformation tbs at the wheel position at each moment
% Because the wheel position does not necessarily coincide with the generated bridge node coordinates
% So linear interpolation is used to calculate the bridge deformation at any position
% Four points cannot form a plane, so triangle interpolation is used
% For specific details, refer to "Modeling and Simulation of Random Excitation Three-Dimensional Pavement Spatial Domain Model"
% ts is the road surface unevenness at the bottom of the wheel at each moment
% bs is the bridge deformation time series, each column represents the bridge deformation at a certain moment
% xv is the x coordinate of the longitudinal bridge direction of the car at each moment
% yv is the y coordinate of the transverse bridge direction of the car at each moment
% node is the bridge node coordinate
% dof_index is the bridge node-degree of freedom mapping relationship
%%%%%%%%%%%%%%%%%%%%%%%%%%
%   D(x1,y2)  C(x2,y2)   %
%   A(x1,y1)  B(x2,y1)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tongji University Bridge Department, Jian Xudong, December 6, 2020

Z_deck=-0.1;
nodeb=node(node(:,3)==Z_deck,:);  
tbz=zeros(1,length(xv));
for i=1:length(xv)
    if i==5625
        
    end
    if xv(i)>=min(nodeb(:,1)) && xv(i)<=max(nodeb(:,1)) && yv(i)>=min(nodeb(:,2)) && yv(i)<=max(nodeb(:,2)) 
        temp=nodeb(nodeb(:,1)<=xv(i),1);temp=sort(temp); 
        x1=temp(end); 
        temp=nodeb(nodeb(:,1)>xv(i),1);temp=sort(temp); 
        if isempty(temp) 
            dofA=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
            dofD=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
            if isempty(dofA) 
                tbz(i)=(0*abs((yv(i)-y2)/(y1-y2))+bz(dofD,i)*abs((yv(i)-y1)/(y1-y2))); 
            elseif isempty(dofD)
                tbz(i)=(bz(dofA,i)*abs((yv(i)-y2)/(y1-y2))+0*abs((yv(i)-y1)/(y1-y2))); 
            else
                tbz(i)=(bz(dofA,i)*abs((yv(i)-y2)/(y1-y2))+bz(dofD,i)*abs((yv(i)-y1)/(y1-y2))); 
            end
            continue 
        else
            x2=temp(1);   
        end
        temp=nodeb(nodeb(:,2)<=yv(i),2);temp=sort(temp);
        y1=temp(end);
        temp=nodeb(nodeb(:,2)>yv(i),2);temp=sort(temp); 
        y2=temp(1);    
        % Calculate the vertical displacement of the bridge deck corresponding to node C
        
        dofC=SearchDofIndex(x2,y2,Z_deck,node,dof_index,3);
        if isempty(dofC)  
            zC=0;
        else
            zC=bz(dofC,i);
        end
        
        % Calculate the vertical displacement of the bridge deck corresponding to node B
        
        dofB=SearchDofIndex(x2,y1,Z_deck,node,dof_index,3);
        if isempty(dofB)  
            zB=0;
        else
            zB=bz(dofB,i);
        end
        
        % Calculate the vertical displacement of the bridge deck corresponding to node A
        
        dofA=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
        if isempty(dofA)  
            zA=0;
        else
            zA=bz(dofA,i);
        end
        
        % Calculate the vertical displacement of the bridge deck corresponding to node D
        dofD=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
        if isempty(dofD)  
            zD=0;
        else
            zD=bz(dofD,i);
        end
        
        tri1_x=[x1,x2,x2]; 
        tri1_y=[y1,y1,y2]; 
        tri2_x=[x1,x2,x1]; 
        tri2_y=[y1,y2,y2];
        if inpolygon(xv(i),yv(i),tri1_x,tri1_y) 
            A=abs(det([1,x1,y1;1,x2,y1;1,x2,y2])/2);  
            A1=abs(det([1,x1,y1;1,x2,y1;1,xv(i),yv(i)])/2); 
            A2=abs(det([1,x2,y1;1,x2,y2;1,xv(i),yv(i)])/2); 
            A3=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2);
            tbz(i)=zC*A1/A+zA*A2/A+zB*A3/A; 
        elseif inpolygon(xv(i),yv(i),tri2_x,tri2_y) 
            A=abs(det([1,x1,y1;1,x2,y2;1,x1,y2])/2);  
            A1=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); 
            A2=abs(det([1,x2,y2;1,x1,y2;1,xv(i),yv(i)])/2); 
            A3=abs(det([1,x1,y2;1,x1,y1;1,xv(i),yv(i)])/2); 
            tbz(i)=zD*A1/A+zA*A2/A+zC*A3/A; 
        end
    end
end
end