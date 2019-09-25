function RRTree = InsertVertex(x_rand,x_min,RRTree,stepsize)
father=x_min(3);
newpoint=[0 0];
theta=atan2(x_rand(1)-x_min(1),x_rand(2)-x_min(2)); 
newPoint = double(int32(x_min(1:2) + stepsize * [sin(theta)  cos(theta)]));
RRTree=[RRTree;[newPoint,father]];
end

