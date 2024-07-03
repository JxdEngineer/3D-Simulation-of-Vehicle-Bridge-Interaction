function node_def=GetDeformation3D(deformation,node,dof_index,scale)
% This function is used to obtain the coordinates of the bridge node after deformation
% node_def is the coordinates of the bridge node after deformation
% deformation is the calculated bridge node displacement, column vector
% node is the coordinates of the bridge node before deformation
% dof_index is the degree of freedom number-node number mapping matrix
% scale is the deformation magnification factor
def=[dof_index,deformation];
node_def=node;

index_X=(def(:,2)==1); %DOF=1, X direction, longitudinal
node_def(def(index_X,1),1)=node_def(def(index_X,1),1)+def(index_X,4)*scale;

index_Y=(def(:,2)==2); %DOF=2, Y direction, lateral
node_def(def(index_Y,1),2)=node_def(def(index_Y,1),2)+def(index_Y,4)*scale;

index_Z=(def(:,2)==3); %DOF=3, Z direction, vertical
node_def(def(index_Z,1),3)=node_def(def(index_Z,1),3)+def(index_Z,4)*scale;
end