function X_near = NearestVertex(x_rand,RRTree)
%NEARESTVERTEX 
dis = distanceCost(RRTree(:,1:2),x_rand);
index=find(min(dis));
X_near=[RRTree(index,1:3),index];
end

