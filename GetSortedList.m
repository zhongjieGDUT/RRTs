function Ls = GetSortedList(x_new,X_near,RRTree)
%GETSORTEDLIST 
[n,~]=size(X_near);
Ls=cell(1,3,n);
step=2;
for i=1:n
    traj = Steer(X_near(i,1:2),x_new,step);
    cost = Cost(RRTree,X_near(i,:))+distanceCost(x_new,X_near(i,1:2));
    Ls{1,1,i} = X_near(i,:);
    Ls{1,2,i} = cost;
    Ls{1,3,i} = traj;   
end
Ls=sortList(Ls);

end


