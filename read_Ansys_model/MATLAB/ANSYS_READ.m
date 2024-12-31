clear
clc
directory='F:\ANSYS MODEL\';       %�����ļ�·��

%% ���벢����node
tic
temp=load([directory,'node.txt']);
node=temp(:,2:4);
save node node
disp(['���벢����node��ʱ',num2str(toc),'��'])
% node��1/2/3�зֱ�Ϊ�ڵ��X/Y/Z����
%% ���벢����element
tic
temp=load([directory,'element.txt']);
element=temp(:,2:5);  %����Ҫ���ݵ�Ԫ�����Ľڵ��Ž����޸ģ�������������
save element element
disp(['���벢����element��ʱ',num2str(toc),'��'])
% element��1/2�зֱ�Ϊ����Ԫ��ͷ�ڵ�����ɶȱ��
%% ���벢����K_Constrained
tic
K_constrained= Matrix_extract('stiff',directory);
save K_constrained K_constrained
disp(['���벢����K_Constrained��ʱ',num2str(toc),'��'])
%% ���벢����M_Constrained
tic
M_constrained= Matrix_extract('mass',directory);
save M_constrained M_constrained
disp(['���벢����M_Constrained��ʱ',num2str(toc),'��'])
%% ���벢�������ɶ�-�ڵ���ӳ���ϵ
tic
dof_index=Dofmapping_extract('stiff',directory);
save dof_index dof_index
disp(['���벢�������ɶ�-�ڵ���ӳ���ϵ',num2str(toc),'��'])
% dof_index��1�У��ڵ��
% dof_index��1�У����ɶ����ͣ�1/2/3/4/5/6����X/Y/Z/RX/RY/RZ��
% dof_index��1�У����ɶȺ�
%% ���벢�������͸նȡ�������������������
tic
[fb,Mnb,Knb] = Modal_FKM_extract('ModalFKM.txt',directory);
Phib=Modal_Phi_extract(dof_index,'ModalPhi.txt',directory);
save fb fb
save Mnb Mnb
save Knb Knb
save Phib Phib
disp(['���벢�������͸նȡ�������������������',num2str(toc),'��'])
%% ����������
clear temp