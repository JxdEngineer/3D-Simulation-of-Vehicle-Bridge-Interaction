function  [fb,Mn,Kn] = Modal_FKM_extract(filename,directory)

% ��ANSYSģ̬�����������ȡģ̬���������MATLAB������Ƶ��fb�����͸ն�Mn����������Kn
% ��Ҫ��ANSYSģ̬�������ʹ�ã������E:\FangCloudV2\personal_space\1 ����\2 ����\%��ʿ����\2
% ��ֵ����\2 ����ģ��\ANSYS��ģ\��������Ŀ¼�µķ�������

path=strcat([directory,filename]);
fid=fopen(path);
Modal=textscan(fid,'%d %f %f %f','HeaderLines',1);
fb=Modal{2};
Mn=Modal{3};
Kn=Modal{4};
end