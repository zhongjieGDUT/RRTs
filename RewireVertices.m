function RRTree_new=RewireVertices(x_min,x_new,Ls,RRTree,map)
%REWIREVERTICES
[~,~,n]=size(Ls);
for i=1:n
    x_near=Ls{1,1,i};
    traj=Ls{1,3,i};
    dis=Cost(RRTree,x_min);
    if dis+distanceCost(x_min(1:2),x_new)+distanceCost(x_new,x_near(1:2))<Cost(RRTree,x_near)
        if checkPath(x_near(1:2),x_new,map)
            RRTree(x_near(4),3)=length(RRTree);
        end
    end
end
RRTree_new=RRTree;
end

