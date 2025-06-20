function  [fb,Mn,Kn] = Modal_FKM_extract(filename,directory)

% 从ANSYS模态分析结果中提取模态分析结果到MATLAB：桥梁频率fb、振型刚度Mn、振型质量Kn
% 需要与ANSYS模态分析配合使用

path=strcat([directory,filename]);
fid=fopen(path);
Modal=textscan(fid,'%d %f %f %f','HeaderLines',1);
fb=Modal{2};
Mn=Modal{3};
Kn=Modal{4};
end
