function  [ Matrix ] = Matrix_extract( mat_type,fname )

%   ��ȡ�ṹ���������նȾ���
%   ����'Stiff'/'Mass'

filename = strcat([fname,'hbfile_',mat_type,'.txt']);
hb_ID = fopen(filename);

% ����ָ��Ĳ���
Mat_pt_para = textscan(hb_ID,'%d',5,'HeaderLines',1);
Col_pt_num = Mat_pt_para{1}(2);                                                   %��ָ������
Row_pt_num = Mat_pt_para{1}(3);                                                   %��ָ������
NZero_num = Mat_pt_para{1}(4);                                                    %�������Ԫ����
RT_num  = Mat_pt_para{1}(5);                                                      %�ұ���������

% ����Ĳ���
Mat_para =  textscan(hb_ID,'%s %d %d %d','HeaderLines',1);
Num_row = double(Mat_para{1,2}(1));                                                     %��������
Num_col = double(Mat_para{1,3}(1));                                                      %��������
fclose(hb_ID);
if RT_num == 0
    LS0 = 4;
else
    LS0 = 5;
end

% ����ָ������
hb_ID = fopen(filename);
Col_pt = textscan(hb_ID,'%d',Col_pt_num,'HeaderLines',LS0);                       %��ָ������
Row_pt = textscan(hb_ID,'%d',Row_pt_num);                                         %��ָ������
NZero = textscan(hb_ID,'%f',NZero_num);                                           %�������Ԫ����
RT = textscan(hb_ID,'%f',RT_num);                                                 %�ұ�������

fclose(hb_ID);

Col_pt = Col_pt{1};
Row_pt = Row_pt{1};
NZero = NZero{1};
RT = RT{1};

%%%����������洢�ľ���
Matrix=zeros(Num_row,Num_col);
% Matrix=sparse(Num_row,Num_col);  %����ϡ�����
for i = 1:Num_col                                                                   %������ѭ��
    sta_col = Col_pt(i);                                                            %�õ���ǰ��ָ��
    end_col = Col_pt(i+1);                                                          %�õ���һ��ָ��
    for j = sta_col:end_col-1                                                       %��ָ��ָ����
        row = Row_pt(j);                                                            %�õ���ǰԪ�ص��к�
        Matrix(row,i) = NZero(j);                                                   %�����кŽ�Ԫ��ֵ���浽������
    end
end

for i = 1:Num_row-1                                                                 %�γ�������Ԫ�أ������õ�������
    for j = i+1:Num_col
        Matrix(i,j) = Matrix(j,i);
    end
end
end



