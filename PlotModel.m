function PlotModel(node,element,expr,bcol,icol,nodes,tris)
% this function is used to plot the finite element model
% Copyright (C) 2004-2012 Per-Olof Persson. See COPYRIGHT.TXT for details.

dim=size(node,2);
switch dim
    case 2
        if nargin<4 | isempty(bcol), bcol=[.8,.9,1]; end
        if nargin<5 | isempty(icol), icol=[0,0,0]; end
        if nargin<6, nodes=0; end
        if nargin<7, tris=0; end
        
        trimesh(element,node(:,1),node(:,2),0*node(:,1),'facecolor',bcol,'edgecolor','k');
        if nodes==1
            line(node(:,1),node(:,2),'linest','none','marker','.','col',icol,'markers',24);
        elseif nodes==2
            for ip=1:size(node,1)
                txtpars={'fontname','times','fontsize',12};
                text(node(ip,1),node(ip,2),num2str(ip),txtpars{:});
            end
        end
        if tris==2
            for it=1:size(element,1)
                pmid=mean(node(element(it,:),:),1);
                txtpars={'fontname','times','fontsize',12,'horizontala','center'};
                text(pmid(1),pmid(2),num2str(it),txtpars{:});
            end
        end
        view(2)
        axis equal
        axis off
        ax=axis;axis(ax*1.001);
    case 3
        if nargin<4 | isempty(bcol), bcol=[.8,.9,1]; end
        if nargin<5 | isempty(icol), icol=[.9,.8,1]; end
        
        if size(element,2)==4
            tri1=surftri(node,element);
            if nargin>2 & ~isempty(expr)
                incl=find(eval(expr));
                element=element(any(ismember(element,incl),2),:);
                tri1=tri1(any(ismember(tri1,incl),2),:);
                tri2=surftri(node,element);
                tri2=setdiff(tri2,tri1,'rows');
                h=trimesh(tri2,node(:,1),node(:,2),node(:,3));
%                 set(h,'facecolor',icol,'edgecolor','k');
                hold on
            end
        else
            tri1=element;
            if nargin>2 & ~isempty(expr)
                incl=find(eval(expr));
                tri1=tri1(any(ismember(tri1,incl),2),:);
            end
        end
        h=trimesh(tri1,node(:,1),node(:,2),node(:,3),'linewidth',2);
        hold off
%         set(h,'edgecolor','b');
        set(gcf,...
            'Unit', 'Centimeter', ...
            'Position', [10, 10, 14.65, 8],...
            'color','w');
        view(3)
        axis equal
        grid off
        %   cameramenu
    otherwise
        error('Unimplemented dimension.');
end
end

function tri=surftri(p,t)
% SURFTRI Find surface triangles from tetrahedra mesh
% TRI=SURFTRI(P,T)

% Copyright (C) 2004-2012 Per-Olof Persson. See COPYRIGHT.TXT for details.

% Form all faces, non-duplicates are surface triangles
faces=[t(:,[1,2,3]);
    t(:,[1,2,4]);
    t(:,[1,3,4]);
    t(:,[2,3,4])];
node4=[t(:,4);t(:,3);t(:,2);t(:,1)];
faces=sort(faces,2);
[~,ix,jx]=unique(faces,'rows');
vec=histc(jx,1:max(jx));
qx=find(vec==1);
tri=faces(ix(qx),:);
node4=node4(ix(qx));

% Orientation
v1=p(tri(:,2),:)-p(tri(:,1),:);
v2=p(tri(:,3),:)-p(tri(:,1),:);
v3=p(node4,:)-p(tri(:,1),:);
ix=find(dot(cross(v1,v2,2),v3,2)>0);
tri(ix,[2,3])=tri(ix,[3,2]);
end