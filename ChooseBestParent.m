function x_min = ChooseBestParent(Ls,map)
%CHOOSEBESTPARENT

x_min=[];
[~,~,n]=size(Ls);
for i=1:n
    traj=Ls{1,3,i};%��ȡ·��
    if checkPath(double(int32(traj(1,:))),double(int32(traj(end,:))),map) %����
        x_min=Ls{1,1,i};
        break;
    end
end

end

