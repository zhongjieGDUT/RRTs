function [x_min,cost] = ChooseBestParentTree(Ls,map)
%CHOOSEBESTPARENTTREE
x_min=[];
cost=inf;
[~,~,n]=size(Ls);
for i=1:n
    traj=Ls{1,3,i};%��ȡ·��
    if checkPath(double(int32(traj(1,:))),double(int32(traj(end,:))),map) %����
        x_min=Ls{1,1,i};
        cost=Ls{1,2,i};
        break;
    end
end

end

