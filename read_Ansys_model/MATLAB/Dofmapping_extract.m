function dof_index=Dofmapping_extract(type,directory)
% 此函数用于提取刚度/质量矩阵中行/列所对应的节点自由度编号
% type可以输入字符'mass'或者'stiff'，基本上用stiff和mass都一样
% Matrix是K或M矩阵
% directory是ANSYS建模文件的路径
% dof_index第一列：节点编号
% dof_index第二列：自由度类型，1-6依次是DX/DY/DZ/RX/RY/RZ
% dof_index第三列：自由度编号
format long g
filepath=[directory,'hbfile_',type,'.mapping'];
fid=fopen(filepath); 
Index_cell=textscan(fid,'%d %d %s','HeaderLines',1);
fclose(fid);
Num_node=length(Index_cell{1});
dof_index=zeros(Num_node,3);
dof_index(:,1)=Index_cell{1,1};                                                
dof_index(:,2)=Index_cell{1,2};                                              
Index_type={};
Dof_type={'UX','UY','UZ','ROTX','ROTY','ROTZ'};
for i=1:Num_node
    Index_type{i,1}=Index_cell{1,3}{i,1};
    dof_index(i,3)=find(ismember(Dof_type,Index_type{i}));                      
end
dof_index(:,1)=[];
dof_index(:,3)=(dof_index(:,1)-1).*6+dof_index(:,2); 
end