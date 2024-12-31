function NodeNum=SearchNode(x1,x2,y1,y2,z1,z2,node)
N=max(size(node));
k=1;
for i=1:N
    if node(i,1)<=x2 && node(i,1)>=x1 ...
         && node(i,2)<=y2 && node(i,2)>=y1 ...
         && node(i,3)<=z2 && node(i,3)>=z1
     NodeNum(k,1)=i;
     k=k+1;
    end
end
end