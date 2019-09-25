function traj=Steer(x_near,x_new,step)
%STEER 
% 
traj=zeros(step,2);
x=linspace(x_near(1),x_new(1),step);
y=linspace(x_near(2),x_new(2),step);
traj(:,1)=x';
traj(:,2)=y';
end

