function  phi = Modal_Phi_extract(dof_index,filename,directory )

% ��ANSYSģ̬�����������ȡģ̬���������MATLAB��������������Phi
% phi�ĵ�n�У���n�����������������ɶȱ��˳������
% ��Ҫ��ANSYSģ̬�������ʹ�ã������E:\FangCloudV2\personal_space\1 ����\2 ����\%��ʿ����\2
% ��ֵ����\2 ����ģ��\ANSYS��ģ\��������Ŀ¼�µķ�������
% ��Ҫ�ȼ�����ڵ�-���ɶȱ��dof_index

path=strcat([directory,filename]);
fid=fopen(path);
Modal=textscan(fid,'%d %d %f %f %f','HeaderLines',1);
order=double(Modal{1}); %���ݸ�ʽ��intת��Ϊdouble��ʽ
orderN=double(max(order)); %���ͽ��������ݸ�ʽ��intת��Ϊdouble��ʽ�������޷�����
nodeN=double(max(Modal{2})); %�ڵ���Ŀ�����ݸ�ʽ��intת��Ϊdouble��ʽ�������޷�����
dofN=length(dof_index);

phi=zeros(dofN,orderN); %����Ԫ������

parpool('local',8) %�򿪲��м�����ٴ�����forѭ��

parfor i=1:orderN
    for j=1:dofN
        dof_type=dof_index(j,2); %��j�����ɶȱ�ŵ�����1/2/3��ʾX/Y/Z
        dof_node=dof_index(j,1); %��j�����ɶȱ�ŵĽڵ��
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