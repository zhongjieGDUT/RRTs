function x_min = ChooseBestParent(Ls,map)
%CHOOSEBESTPARENT

x_min=[];
[~,~,n]=size(Ls);
for i=1:n
    traj=Ls{1,3,i};%获取路径
    if checkPath(double(int32(traj(1,:))),double(int32(traj(end,:))),map) %无碰
        x_min=Ls{1,1,i};
        break;
    end
end

end

