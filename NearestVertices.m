function X_near = NearestVertices(x_new,RRTree)
% NearestVertices 
% ������RRTree�о�����������r���ڵĵ�
% ��¼�����꣬�����е����꣬���ڵ㡣
[num,~]=size(RRTree);
gama =500;
r = gama*sqrt(log(num)/num);
dis = distanceCost(RRTree(:,1:2),x_new);
index = find(dis<=r);
[n,~] = size(index);
if n==0
    X_near=[];
else
    X_near=zeros(n,4);
    for i=1:n
        X_near(i,:)=[RRTree(index(i),1:3),index(i)];
    end
end
end

