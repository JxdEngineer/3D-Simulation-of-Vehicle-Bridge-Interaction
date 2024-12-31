function  [ Matrix ] = Matrix_extract( mat_type,fname )

%   提取结构的质量、刚度矩阵
%   输入'Stiff'/'Mass'

filename = strcat([fname,'hbfile_',mat_type,'.txt']);
hb_ID = fopen(filename);

% 矩阵指针的参数
Mat_pt_para = textscan(hb_ID,'%d',5,'HeaderLines',1);
Col_pt_num = Mat_pt_para{1}(2);                                                   %列指针总数
Row_pt_num = Mat_pt_para{1}(3);                                                   %行指针总数
NZero_num = Mat_pt_para{1}(4);                                                    %矩阵非零元总数
RT_num  = Mat_pt_para{1}(5);                                                      %右边项总行数

% 矩阵的参数
Mat_para =  textscan(hb_ID,'%s %d %d %d','HeaderLines',1);
Num_row = double(Mat_para{1,2}(1));                                                     %矩阵行数
Num_col = double(Mat_para{1,3}(1));                                                      %矩阵列数
fclose(hb_ID);
if RT_num == 0
    LS0 = 4;
else
    LS0 = 5;
end

% 建立指针数组
hb_ID = fopen(filename);
Col_pt = textscan(hb_ID,'%d',Col_pt_num,'HeaderLines',LS0);                       %列指针数组
Row_pt = textscan(hb_ID,'%d',Row_pt_num);                                         %行指针数组
NZero = textscan(hb_ID,'%f',NZero_num);                                           %矩阵非零元数组
RT = textscan(hb_ID,'%f',RT_num);                                                 %右边项数组

fclose(hb_ID);

Col_pt = Col_pt{1};
Row_pt = Row_pt{1};
NZero = NZero{1};
RT = RT{1};

%%%定义满矩阵存储的矩阵
Matrix=zeros(Num_row,Num_col);
% Matrix=sparse(Num_row,Num_col);  %定义稀疏矩阵
for i = 1:Num_col                                                                   %以列数循环
    sta_col = Col_pt(i);                                                            %得到当前列指针
    end_col = Col_pt(i+1);                                                          %得到下一列指针
    for j = sta_col:end_col-1                                                       %列指针指向行
        row = Row_pt(j);                                                            %得到当前元素的行号
        Matrix(row,i) = NZero(j);                                                   %按行列号将元素值保存到矩阵中
    end
end

for i = 1:Num_row-1                                                                 %形成上三角元素，进而得到满矩阵
    for j = i+1:Num_col
        Matrix(i,j) = Matrix(j,i);
    end
end
end



