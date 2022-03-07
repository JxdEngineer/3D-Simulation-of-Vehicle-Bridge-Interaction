function Fb=GetBridgeNodalF(xv,yv,node,dof_index,Fv)
% �˺������ڼ��㳵�����ط��䵽�����ṹ�ϵĽڵ����Fb
% xv��ÿһʱ����������������x����
% yv��ÿһʱ���������ں�����Y����
% node��ANSYS���뵽MATLAB�еĽ������
% dof_index��ANSYS���뵽MATLAB�еĽڵ�����-���ɶ�ӳ���ϵ
% Fv�ǳ��ֺ���ʱ������
% ��ͼ�ǳ������������������ʾ��ͼ�����ھ��뷴�ȷ��������
%%%%%%%%%%%%%%%%%%%%%%%%%%
%   4(x1,y2)  1(x2,y2)   %
%   3(x1,y1)  2(x2,y1)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% ͬ�ô�ѧ����ϵ�����񶫣�2020��12��06��

Z_deck=-0.1;
nodeb=node(node(:,3)==Z_deck,:);  %����ڵ��Z����ΪZ_deck
% Ϊʲô���ﲻֱ��дnode=node(node(:,3)==0,:)�أ���Ϊ������ı�����ţ����������SearchDofIndex����
Fb=zeros(length(dof_index),length(xv));
for i=1:length(xv)
    if i==250
        
    end
    if xv(i)>=min(nodeb(:,1)) && xv(i)<=max(nodeb(:,1)) && yv(i)>=min(nodeb(:,2)) && yv(i)<=max(nodeb(:,2)) %��ʱ���ֹ켣��������Χ��
        temp=nodeb(nodeb(:,1)<=xv(i),1);temp=sort(temp); %�������ȡ�߽�ֵ
        x1=temp(end); %��iʱ�̳������������ڵ��ı�������������������½�
        temp=nodeb(nodeb(:,1)>xv(i),1);temp=sort(temp); %�������ȡ�߽�ֵ
        if isempty(temp)   %tempΪ��ʱ������ǡ�������������߽�ڵ��ϣ�x1Ϊ�����߽�ڵ������꣬x2Ϊ��������߽�ڵ�������꣨ע�������м򻯣�û����CP�ڵ㡣���ǶԽ��Ӧ��Ӱ�첻����Ϊ��ֻ�����һ�����ز���
            dof3=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
            dof4=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
            if isempty(dof3) %��ʱ�ڵ����ɶȱ�Լ��
                Fb(dof4,i)=Fv(i)*abs((yv(i)-y1)/(y2-y1));
            elseif isempty(dof4)
                Fb(dof3,i)=Fv(i)*abs((yv(i)-y2)/(y2-y1)); %��ʱ��λ��λ��ʹ��һά���뷴�ȷ����㣬ֻ�迼��y����
            else
                Fb(dof3,i)=Fv(i)*abs((yv(i)-y2)/(y2-y1)); %��ʱ��λ��λ��ʹ��һά���뷴�ȷ����㣬ֻ�迼��y����
                Fb(dof4,i)=Fv(i)*abs((yv(i)-y1)/(y2-y1));
            end
            continue  %��������ѭ����ֱ�ӽ�����һ��
        else
            x2=temp(1);    %��iʱ�̳������������ڵ��ı�������������������Ͻ磬����û�еȺţ�����ڵ��غ��������
        end
        temp=nodeb(nodeb(:,2)<=yv(i),2);temp=sort(temp); %�������ȡ�߽�ֵ
        y1=temp(end); %��iʱ�̳������������ڵ��ı�������ĺ����������½�
        temp=nodeb(nodeb(:,2)>yv(i),2);temp=sort(temp); %�������ȡ�߽�ֵ
        y2=temp(1);    %��iʱ�̳������������ڵ��ı�������ĺ����������Ͻ�
        % ����ڵ�1���䵽�Ľڵ����
        dof1=SearchDofIndex(x2,y2,Z_deck,node,dof_index,3);
        if ~isempty(dof1)  %dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ�������ü���
            Fdof1=Fv(i)*abs((xv(i)-x1)/(x2-x1)*(yv(i)-y1)/(y2-y1)); %���뷴�ȷ���������
            Fb(dof1,i)=Fdof1;
        end
        % ����ڵ�2���䵽�Ľڵ����
        dof2=SearchDofIndex(x2,y1,Z_deck,node,dof_index,3);
        if ~isempty(dof2)  %dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ�������ü���
            Fdof2=Fv(i)*abs((xv(i)-x1)/(x2-x1)*(yv(i)-y2)/(y2-y1)); %���뷴�ȷ���������
            Fb(dof2,i)=Fdof2;
        end
        % ����ڵ�3���䵽�Ľڵ����
        dof3=SearchDofIndex(x1,y1,Z_deck,node,dof_index,3);
        if ~isempty(dof3)  %dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ�������ü���
            Fdof3=Fv(i)*abs((xv(i)-x2)/(x2-x1)*(yv(i)-y2)/(y2-y1)); %���뷴�ȷ���������
            Fb(dof3,i)=Fdof3;
        end
        % ����ڵ�4���䵽�Ľڵ����
        dof4=SearchDofIndex(x1,y2,Z_deck,node,dof_index,3);
        if ~isempty(dof4)  %dofΪ�ռ�ʱ�ýڵ����ɶȲ����ڣ�������ģ�ʹ˴����ڱ߽�Լ�������ü���
            Fdof4=Fv(i)*abs((xv(i)-x2)/(x2-x1)*(yv(i)-y1)/(y2-y1)); %���뷴�ȷ���������
            Fb(dof4,i)=Fdof4;
        end
    end
end