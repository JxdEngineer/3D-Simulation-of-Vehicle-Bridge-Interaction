function tbz=GetTireDispBridge3D(bz,xv,yv,node,dof_index)
% �˺���������ÿʱ�̳�������λ�ô�����������tbs
% ��Ϊ����λ�ò�һ�������ɵ�������������غ�
% ���Բ������Բ�ֵ����������λ�ô�����������
% �ĵ㲻���γ�һ��ƽ�棬�ʲ��������β�ֵ��
% ����ϸ�ڲο������������ά·��ռ���ģ�ͽ�ģ����桷
% ts��ÿһʱ�̳��ֵײ���·�治ƽ��
% bs����������ʱ�����У�ÿһ�б�ʾĳʱ�̵���������
% xv��ÿһʱ����������������x����
% yv��ÿһʱ���������ں�����Y����
% node�������������
% dof_index���������-���ɶ�ӳ���ϵ
%%%%%%%%%%%%%%%%%%%%%%%%%%
%   D(x1,y2)  C(x2,y2)   %
%   A(x1,y1)  B(x2,y1)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% ͬ�ô�ѧ����ϵ�����񶫣�2020��12��06��

Z_deck=-0.1;
nodeb=node(node(:,3)==Z_deck,:);  %��������ֻʹ��node��Z����ΪZ_deck�ĵ㣬����Ѱ��������ϵĵ�
% Ϊʲô���ﲻֱ��дnode=node(node(:,3)==0,:)�أ���Ϊ������ı�����ţ����������SearchDofIndex����
tbz=zeros(1,length(xv));
for i=1:length(xv)
    if i==5625
        
    end
    if xv(i)>=min(nodeb(:,1)) && xv(i)<=max(nodeb(:,1)) && yv(i)>=min(nodeb(:,2)) && yv(i)<=max(nodeb(:,2)) %��ʱ���ֹ켣��������Χ��
        temp=nodeb(nodeb(:,1)<=xv(i),1);temp=sort(temp); %�������ȡ�߽�ֵ
        x1=temp(end); %��iʱ�̳������������ڵ��ı�������������������½�
        temp=nodeb(nodeb(:,1)>xv(i),1);temp=sort(temp); %�������ȡ�߽�ֵ
        if isempty(temp) %tempΪ��ʱ������ǡ�������������߽�ڵ��ϣ�x1Ϊ�����߽�ڵ������꣬x2Ϊ��������߽�ڵ�������꣨ע�������м򻯣�û����CP�ڵ㡣���ǶԽ��Ӧ��Ӱ�첻����Ϊ��ֻ�����һ�����ز���
            dofA=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
            dofD=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
            if isempty(dofA)  %��ʱ�б߽�Լ�����ڵ����ɶȲ�����
                tbz(i)=(0*abs((yv(i)-y2)/(y1-y2))+bz(dofD,i)*abs((yv(i)-y1)/(y1-y2))); %��ʱ��λ��λ��ʹ��һά���뷴�ȷ����㣬ֻ�迼��y����
            elseif isempty(dofD)
                tbz(i)=(bz(dofA,i)*abs((yv(i)-y2)/(y1-y2))+0*abs((yv(i)-y1)/(y1-y2))); %��ʱ��λ��λ��ʹ��һά���뷴�ȷ����㣬ֻ�迼��y����
            else
                tbz(i)=(bz(dofA,i)*abs((yv(i)-y2)/(y1-y2))+bz(dofD,i)*abs((yv(i)-y1)/(y1-y2))); %��ʱ��λ��λ��ʹ��һά���뷴�ȷ����㣬ֻ�迼��y����
            end
            continue  %��������ѭ����ֱ�ӽ�����һ��
        else
            x2=temp(1);    %��iʱ�̳������������ڵ��ı�������������������Ͻ磬����û�еȺţ�����ڵ��غ��������
        end
        temp=nodeb(nodeb(:,2)<=yv(i),2);temp=sort(temp); %�������ȡ�߽�ֵ
        y1=temp(end); %��iʱ�̳������������ڵ��ı�������ĺ����������½�
        temp=nodeb(nodeb(:,2)>yv(i),2);temp=sort(temp); %�������ȡ�߽�ֵ
        y2=temp(1);    %��iʱ�̳������������ڵ��ı�������ĺ����������Ͻ�
        %����ڵ�C��Ӧ����������λ��
        
        dofC=SearchDofIndex(x2,y2,Z_deck,node,dof_index,3);
        if isempty(dofC)  %dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ����λ��Ϊ0
            zC=0;
        else
            zC=bz(dofC,i);
        end
        
        %����ڵ�B��Ӧ����������λ��
        
        dofB=SearchDofIndex(x2,y1,Z_deck,node,dof_index,3);
        if isempty(dofB)  %CP��ֻ��һ���ڵ���dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ����λ��Ϊ0
            zB=0;
        else
            zB=bz(dofB,i);
        end
        
        %����ڵ�A��Ӧ����������λ��
        
        dofA=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
        if isempty(dofA)  %CP��ֻ��һ���ڵ���dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ����λ��Ϊ0
            zA=0;
        else
            zA=bz(dofA,i);
        end
        
        %����ڵ�D��Ӧ����������λ��
        dofD=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
        if isempty(dofD)  %CP��ֻ��һ���ڵ���dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ����λ��Ϊ0
            zD=0;
        else
            zD=bz(dofD,i);
        end
        
        tri1_x=[x1,x2,x2]; %������ABC�����ǵ��X���꣬������ʾ��ͼ����
        tri1_y=[y1,y1,y2]; %������ABC�����ǵ��Y����
        tri2_x=[x1,x2,x1]; %������ACD�����ǵ��X����
        tri2_y=[y1,y2,y2]; %������ACD�����ǵ��Y����
        if inpolygon(xv(i),yv(i),tri1_x,tri1_y) %��ʱ���ֹ켣��P����������ABC��
            A=abs(det([1,x1,y1;1,x2,y1;1,x2,y2])/2);  %������ABC�����
            A1=abs(det([1,x1,y1;1,x2,y1;1,xv(i),yv(i)])/2); %������ABP�����
            A2=abs(det([1,x2,y1;1,x2,y2;1,xv(i),yv(i)])/2); %������BCP�����
            A3=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); %������CAP�����
            tbz(i)=zC*A1/A+zA*A2/A+zB*A3/A; %�����β�ֵ���ο������޵�Ԫ����P106
        elseif inpolygon(xv(i),yv(i),tri2_x,tri2_y) %��ʱ���ֹ켣������������ACD��
            A=abs(det([1,x1,y1;1,x2,y2;1,x1,y2])/2);  %������ACD�����
            A1=abs(det([1,x1,y1;1,x2,y2;1,xv(i),yv(i)])/2); %������ACP�����
            A2=abs(det([1,x2,y2;1,x1,y2;1,xv(i),yv(i)])/2); %������CDP�����
            A3=abs(det([1,x1,y2;1,x1,y1;1,xv(i),yv(i)])/2); %������DAP�����
            tbz(i)=zD*A1/A+zA*A2/A+zC*A3/A; %�����β�ֵ���ο������޵�Ԫ����P106
        end
    end
end
end