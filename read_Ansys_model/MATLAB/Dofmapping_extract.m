function dof_index=Dofmapping_extract(type,directory)
% �˺���������ȡ�ն�/������������/������Ӧ�Ľڵ����ɶȱ��
% type���������ַ�'mass'����'stiff'����������stiff��mass��һ��
% Matrix��K��M����
% directory��ANSYS��ģ�ļ���·��
% dof_index��һ�У��ڵ���
% dof_index�ڶ��У����ɶ����ͣ�1-6������DX/DY/DZ/RX/RY/RZ
% dof_index�����У����ɶȱ��
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