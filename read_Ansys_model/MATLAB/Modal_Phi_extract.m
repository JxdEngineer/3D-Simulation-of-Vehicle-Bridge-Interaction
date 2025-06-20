function  phi = Modal_Phi_extract(dof_index,filename,directory )

% 从ANSYS模态分析结果中提取模态分析结果到MATLAB：桥梁振型向量Phi
% phi的第n列：第n阶振型向量，按自由度编号顺序排序
% 需要与ANSYS模态分析配合使用
% 需要先计算出节点-自由度编号dof_index

path=strcat([directory,filename]);
fid=fopen(path);
Modal=textscan(fid,'%d %d %f %f %f','HeaderLines',1);
order=double(Modal{1}); %数据格式从int转化为double形式
orderN=double(max(order)); %振型阶数，数据格式从int转化为double形式，否则无法计算
nodeN=double(max(Modal{2})); %节点数目，数据格式从int转化为double形式，否则无法计算
dofN=length(dof_index);

phi=zeros(dofN,orderN); %振型元胞数组

parpool('local',8) %打开并行计算加速大体量for循环

parfor i=1:orderN
    for j=1:dofN
        dof_type=dof_index(j,2); %第j个自由度编号的类型1/2/3表示X/Y/Z
        dof_node=dof_index(j,1); %第j个自由度编号的节点号
        if dof_type==1
            phi(j,i)=Modal{3}((i-1)*nodeN+dof_node);
        elseif dof_type==2
            phi(j,i)=Modal{4}((i-1)*nodeN+dof_node);
        elseif dof_type==3
            phi(j,i)=Modal{5}((i-1)*nodeN+dof_node);
        end
    disp(['i=',num2str(i),',j=',num2str(j)])    
    end
end
end
