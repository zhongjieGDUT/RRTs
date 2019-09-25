function [flag,RRTree]=ConnectGraphs(RRTree,x1,x2,stepsize,map)
theta=atan2(x1(1)-x2(1),x1(2)-x2(2));
x_new = double(int32(x2(1:2) + stepsize * [sin(theta)  cos(theta)]));
X_near=NearestVertices(x_new,RRTree);
Ls = GetSortedList(x1,X_near,RRTree);
x_min = ChooseBestParent(Ls,map);
traj=[];
flag=0;
if ~size(x_min)==[0 0]
    RRTree=[RRTree;[x1(1:2),x_min(4)]];
    flag=1;
end
end

