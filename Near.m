function X_near = Near(RRTree,x_new)
%NEAR 
[num,~]=size(RRTree);
gama = 30;
yita= 30;
r = min(gama*sqrt(log(num+1)/num),yita)
dis = distanceCost(RRTree(:,1:2),x_new);
index = find(dis<=r);
[n,~] = size(index);
if n==0
    X_near=[];
else
    X_near=zeros(n,3);
    for i=1:n
        X_near(i,:)=[RRTree(index(i),1:2),index(i)];
    end
end

end

