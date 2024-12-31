function node_def=GetDeformation3D(deformation,node,dof_index,scale)
% 此函数用于获取变形后的桥梁节点坐标
% node_def是变形后的桥梁节点坐标
% deformation是求解出的桥梁节点位移，列向量
% node是变形前的桥梁节点坐标
% dof_index是自由度编号-节点号映射矩阵
% scale是变形放大因子
def=[dof_index,deformation];
node_def=node;
for i=1:length(dof_index)
    cp=SearchNode(node(def(i,1),1)-0.05,node(def(i,1),1)+0.05,...
        node(def(i,1),2)-0.05,node(def(i,1),2)+0.05,...
        node(def(i,1),3)-0.05,node(def(i,1),3)+0.05,...
        node);  %此处额外定义cp是为了令相邻耦合节点的位移相等（用于铰接板梁模型）
    if length(cp)==1 %此时只有1个节点，不存在节点耦合
        if def(i,2)==1  %如果自由度类型为1，即X方向，纵桥向
            node_def(def(i,1),1)=node_def(def(i,1),1)+def(i,4)*scale;
        elseif def(i,2)==2  %如果自由度类型为2，即Y方向，横桥向
            node_def(def(i,1),2)=node_def(def(i,1),2)+def(i,4)*scale;
        elseif def(i,2)==3  %如果自由度类型为3，即Z方向，竖桥向
            node_def(def(i,1),3)=node_def(def(i,1),3)+def(i,4)*scale;
        end
    elseif length(cp)==2 %此时有2个节点，存在节点耦合
        if def(i,2)==1 && def(i,4)~=0  %如果自由度类型为1，即X方向，纵桥向。不等于0判断用于给耦合节点的位移赋值
            node_def(cp(1),1)=node_def(def(i,1),1)+def(i,4)*scale;
            node_def(cp(2),1)=node_def(def(i,1),1)+def(i,4)*scale;
        elseif def(i,2)==2  %如果自由度类型为2，即Y方向，横桥向
            node_def(cp(1),2)=node_def(def(i,1),2)+def(i,4)*scale;
            node_def(cp(2),2)=node_def(def(i,1),2)+def(i,4)*scale;
        elseif def(i,2)==3  %如果自由度类型为3，即Z方向，竖桥向
            node_def(cp(1),3)=node_def(def(i,1),3)+def(i,4)*scale;
            node_def(cp(2),3)=node_def(def(i,1),3)+def(i,4)*scale;
        end
    else
        disp('cp节点数大于2，请检查')
    end
end