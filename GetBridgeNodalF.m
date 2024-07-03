function Fb=GetBridgeNodalF(xv,yv,node,dof_index,Fv)
% This function is used to calculate the node load Fb distributed to the bridge structure by the vehicle load
% xv is the x coordinate of the longitudinal bridge where the vehicle is located at each moment
% yv is the Y coordinate of the transverse bridge where the vehicle is located at each moment
% node is the node coordinate imported from ANSYS to MATLAB
% dof_index is the node coordinate-degree of freedom mapping relationship imported from ANSYS to MATLAB
% Fv is the wheel load time series
% The figure below is a schematic diagram of the bridge node grid where the wheel is located, used to distribute the load using the inverse distance method
%%%%%%%%%%%%%%%%%%%%%%%%%%
%   4(x1,y2)  1(x2,y2)   %
%   3(x1,y1)  2(x2,y1)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Department of Bridge Engineering, Tongji University, Jian Xudong, December 6, 2020

Z_deck=-0.1;
nodeb=node(node(:,3)==Z_deck,:);  %The Z coordinate of the bridge deck node is Z_deck
Fb=zeros(length(dof_index),length(xv));
for i=1:length(xv)
    if i==250
        
    end
    if xv(i)>=min(nodeb(:,1)) && xv(i)<=max(nodeb(:,1)) && yv(i)>=min(nodeb(:,2)) && yv(i)<=max(nodeb(:,2)) %At this time, the wheel track is within the bridge range
        temp=nodeb(nodeb(:,1)<=xv(i),1);temp=sort(temp); % Sorting makes it easier to get boundary values
        x1=temp(end); % The lower bound of the longitudinal coordinate of the quadrilateral mesh of the bridge node where the wheel is located at the i-th moment
        temp=nodeb(nodeb(:,1)>xv(i),1);temp=sort(temp); % Sorting makes it easier to get boundary values
        if isempty(temp) % When temp is empty, the wheel acts exactly on the bridge boundary node, x1 is the vertical coordinate of the bridge boundary node, and x2 is the vertical coordinate of the node closest to the bridge boundary (note that there is a simplification here, and the CP node is not considered. However, it should not have much impact on the result because this is only the last load step)
            dof3=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
            dof4=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
            if isempty(dof3) %At this time, the node degrees of freedom are constrained
                Fb(dof4,i)=Fv(i)*abs((yv(i)-y1)/(y2-y1));
            elseif isempty(dof4)
                Fb(dof3,i)=Fv(i)*abs((yv(i)-y2)/(y2-y1)); %At this time, the displacement of the wheel position is calculated using the one-dimensional distance inverse method, and only y needs to be considered
            else
                Fb(dof3,i)=Fv(i)*abs((yv(i)-y2)/(y2-y1)); %At this time, the displacement of the wheel position is calculated using the one-dimensional distance inverse method, and only y needs to be considered
                Fb(dof4,i)=Fv(i)*abs((yv(i)-y1)/(y2-y1));
            end
            continue % Jump out of this loop and go directly to the next one
        else
            x2=temp(1);   % The upper bound of the longitudinal coordinates of the quadrilateral grid of the bridge node where the wheel is located at the i-th moment. There is no equal sign here to avoid node overlap.
        end
        temp=nodeb(nodeb(:,2)<=yv(i),2);temp=sort(temp); % Sorting makes it easier to get boundary values
        y1=temp(end);  % The lower bound of the transverse coordinate of the quadrilateral mesh of the bridge node where the wheel is located at the i-th moment
        temp=nodeb(nodeb(:,2)>yv(i),2);temp=sort(temp); % Sorting makes it easier to get boundary values
        y2=temp(1);    %The upper bound of the transverse coordinate of the quadrilateral mesh of the bridge node where the wheel is located at the i-th moment
        % Calculate the node load assigned to node 1
        dof1=SearchDofIndex(x2,y2,Z_deck,node,dof_index,3);
        if ~isempty(dof1)  %When dof is an empty set, the node degree of freedom does not exist, that is, the bridge model has boundary constraints here and no force is required.
            Fdof1=Fv(i)*abs((xv(i)-x1)/(x2-x1)*(yv(i)-y1)/(y2-y1)); % Calculate node forces using the inverse distance method
            Fb(dof1,i)=Fdof1;
        end
        % Calculate the node load assigned to node 2
        dof2=SearchDofIndex(x2,y1,Z_deck,node,dof_index,3);
        if ~isempty(dof2) 
            Fdof2=Fv(i)*abs((xv(i)-x1)/(x2-x1)*(yv(i)-y2)/(y2-y1)); 
            Fb(dof2,i)=Fdof2;
        end
        % Calculate the node load assigned to node 3
        dof3=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
        if ~isempty(dof3)  
            Fdof3=Fv(i)*abs((xv(i)-x2)/(x2-x1)*(yv(i)-y2)/(y2-y1));
            Fb(dof3,i)=Fdof3;
        end
        % Calculate the node load assigned to node 4
        dof4=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
        if ~isempty(dof4)  
            Fdof4=Fv(i)*abs((xv(i)-x2)/(x2-x1)*(yv(i)-y1)/(y2-y1));
            Fb(dof4,i)=Fdof4;
        end
    end
end