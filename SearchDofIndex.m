function dof=SearchDofIndex(x,y,z,node,dof_index,direc)
% this function is used to find out the DOF index of a point belonging to
% the bridge model
% dof£º
% x/y/z£ºx/y/z coordinates of the node
% node£ºnode information of the ANSYS model
% dof_index£ºDOF index of the ANSYS model
% direc£ºX/Y/Z/UX/UY/UZ direction, denoted as 1/2/3/4/5/6
% author: Jian Xudong, Tongji University
NodeNum=SearchNode(x-0.001,x+0.001,...
    y-0.001,y+0.001,...
    z-0.001,z+0.001,node);
temp=[dof_index(dof_index(:,1)==NodeNum,:),find(dof_index(:,1)==NodeNum)];
dof=temp(temp(:,2)==direc,4);
end

function NodeNum=SearchNode(x1,x2,y1,y2,z1,z2,node)
N=max(size(node));
k=1;
for i=1:N
    if node(i,1)<=x2 && node(i,1)>=x1 ...
         && node(i,2)<=y2 && node(i,2)>=y1 ...
         && node(i,3)<=z2 && node(i,3)>=z1
     NodeNum(k,1)=i;
     k=k+1;
    end
end
end