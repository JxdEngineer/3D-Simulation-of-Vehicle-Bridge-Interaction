function node_def=GetDeformation3D(deformation,node,dof_index,scale)
% �˺������ڻ�ȡ���κ�������ڵ�����
% node_def�Ǳ��κ�������ڵ�����
% deformation�������������ڵ�λ�ƣ�������
% node�Ǳ���ǰ�������ڵ�����
% dof_index�����ɶȱ��-�ڵ��ӳ�����
% scale�Ǳ��ηŴ�����
def=[dof_index,deformation];
node_def=node;

index_X=(def(:,2)==1);%���ɶ�����Ϊ1����X����������
node_def(def(index_X,1),1)=node_def(def(index_X,1),1)+def(index_X,4)*scale;

index_Y=(def(:,2)==2);%���ɶ�����Ϊ2����Y���򣬺�����
node_def(def(index_Y,1),2)=node_def(def(index_Y,1),2)+def(index_Y,4)*scale;

index_Z=(def(:,2)==3); %���ɶ�����Ϊ3����Z����������
node_def(def(index_Z,1),3)=node_def(def(index_Z,1),3)+def(index_Z,4)*scale;
end