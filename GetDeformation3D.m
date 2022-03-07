function node_def=GetDeformation3D(deformation,node,dof_index,scale)
% 此函数用于获取变形后的桥梁节点坐标
% node_def是变形后的桥梁节点坐标
% deformation是求解出的桥梁节点位移，列向量
% node是变形前的桥梁节点坐标
% dof_index是自由度编号-节点号映射矩阵
% scale是变形放大因子
def=[dof_index,deformation];
node_def=node;

index_X=(def(:,2)==1);%自由度类型为1，即X方向，纵桥向
node_def(def(index_X,1),1)=node_def(def(index_X,1),1)+def(index_X,4)*scale;

index_Y=(def(:,2)==2);%自由度类型为2，即Y方向，横桥向
node_def(def(index_Y,1),2)=node_def(def(index_Y,1),2)+def(index_Y,4)*scale;

index_Z=(def(:,2)==3); %自由度类型为3，即Z方向，竖桥向
node_def(def(index_Z,1),3)=node_def(def(index_Z,1),3)+def(index_Z,4)*scale;
end