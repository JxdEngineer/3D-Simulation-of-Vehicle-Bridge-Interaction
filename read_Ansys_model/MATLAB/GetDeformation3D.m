function node_def=GetDeformation3D(deformation,node,dof_index,scale)
% �˺������ڻ�ȡ���κ�������ڵ�����
% node_def�Ǳ��κ�������ڵ�����
% deformation�������������ڵ�λ�ƣ�������
% node�Ǳ���ǰ�������ڵ�����
% dof_index�����ɶȱ��-�ڵ��ӳ�����
% scale�Ǳ��ηŴ�����
def=[dof_index,deformation];
node_def=node;
for i=1:length(dof_index)
    cp=SearchNode(node(def(i,1),1)-0.05,node(def(i,1),1)+0.05,...
        node(def(i,1),2)-0.05,node(def(i,1),2)+0.05,...
        node(def(i,1),3)-0.05,node(def(i,1),3)+0.05,...
        node);  %�˴����ⶨ��cp��Ϊ����������Ͻڵ��λ����ȣ����ڽ½Ӱ���ģ�ͣ�
    if length(cp)==1 %��ʱֻ��1���ڵ㣬�����ڽڵ����
        if def(i,2)==1  %������ɶ�����Ϊ1����X����������
            node_def(def(i,1),1)=node_def(def(i,1),1)+def(i,4)*scale;
        elseif def(i,2)==2  %������ɶ�����Ϊ2����Y���򣬺�����
            node_def(def(i,1),2)=node_def(def(i,1),2)+def(i,4)*scale;
        elseif def(i,2)==3  %������ɶ�����Ϊ3����Z����������
            node_def(def(i,1),3)=node_def(def(i,1),3)+def(i,4)*scale;
        end
    elseif length(cp)==2 %��ʱ��2���ڵ㣬���ڽڵ����
        if def(i,2)==1 && def(i,4)~=0  %������ɶ�����Ϊ1����X���������򡣲�����0�ж����ڸ���Ͻڵ��λ�Ƹ�ֵ
            node_def(cp(1),1)=node_def(def(i,1),1)+def(i,4)*scale;
            node_def(cp(2),1)=node_def(def(i,1),1)+def(i,4)*scale;
        elseif def(i,2)==2  %������ɶ�����Ϊ2����Y���򣬺�����
            node_def(cp(1),2)=node_def(def(i,1),2)+def(i,4)*scale;
            node_def(cp(2),2)=node_def(def(i,1),2)+def(i,4)*scale;
        elseif def(i,2)==3  %������ɶ�����Ϊ3����Z����������
            node_def(cp(1),3)=node_def(def(i,1),3)+def(i,4)*scale;
            node_def(cp(2),3)=node_def(def(i,1),3)+def(i,4)*scale;
        end
    else
        disp('cp�ڵ�������2������')
    end
end