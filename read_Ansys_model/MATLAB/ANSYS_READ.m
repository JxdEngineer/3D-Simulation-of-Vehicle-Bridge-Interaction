clear
clc
directory='F:\ANSYS MODEL\';       %定义文件路径

%% 读入并保存node
tic
temp=load([directory,'node.txt']);
node=temp(:,2:4);
save node node
disp(['读入并保存node耗时',num2str(toc),'秒'])
% node第1/2/3列分别为节点的X/Y/Z坐标
%% 读入并保存element
tic
temp=load([directory,'element.txt']);
element=temp(:,2:5);  %这里要根据单元包含的节点编号进行修改！！！！！！！
save element element
disp(['读入并保存element耗时',num2str(toc),'秒'])
% element第1/2列分别为梁单元两头节点的自由度编号
%% 读入并保存K_Constrained
tic
K_constrained= Matrix_extract('stiff',directory);
save K_constrained K_constrained
disp(['读入并保存K_Constrained耗时',num2str(toc),'秒'])
%% 读入并保存M_Constrained
tic
M_constrained= Matrix_extract('mass',directory);
save M_constrained M_constrained
disp(['读入并保存M_Constrained耗时',num2str(toc),'秒'])
%% 读入并保存自由度-节点编号映射关系
tic
dof_index=Dofmapping_extract('stiff',directory);
save dof_index dof_index
disp(['读入并保存自由度-节点编号映射关系',num2str(toc),'秒'])
% dof_index第1列：节点号
% dof_index第1列：自由度类型（1/2/3/4/5/6代表X/Y/Z/RX/RY/RZ）
% dof_index第1列：自由度号
%% 读入并保存振型刚度、振型质量、振型向量
tic
[fb,Mnb,Knb] = Modal_FKM_extract('ModalFKM.txt',directory);
Phib=Modal_Phi_extract(dof_index,'ModalPhi.txt',directory);
save fb fb
save Mnb Mnb
save Knb Knb
save Phib Phib
disp(['读入并保存振型刚度、振型质量、振型向量',num2str(toc),'秒'])
%% 清除多余变量
clear temp