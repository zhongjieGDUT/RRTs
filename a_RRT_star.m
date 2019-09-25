% ? Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 

% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
% clear;
close all;
% 原来是RRT，现在将其修改为RRT*
% 参考论文：
% 《Intelligent bidirectional rapidly-exploring random trees for optimal
% motion planning in complex cluttered environments》
% Algrithm 5: RRT*
% 作者:钟 杰
% 2019/9/3

map=imbinarize(imread('map2.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same

display=true; %display of RRT
N=160000;
%%%%% parameters end here %%%%%


if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end
RRTree=double([source -1]); % RRT rooted at the source, representation node and parent index
failedAttempts=0;
counter=0;
pathFound=true;
cost_best=1000;pl=zeros(1,N);
tic;
for i=1:1:N  %loop to grow RRTs
    pl(i)=cost_best;
    i
    % RRTree 的第三列数字代表该节点的父节点的坐标
    if rand <= 0.5 
        x_rand=rand(1,2) .* size(map); % random sample
    else
        x_rand=goal; % sample taken as goal to bias tree generation to goal
    end
%   A 是距离x_rand最近的节点距离，I是其索引，
    [~, I]=min(distanceCost(RRTree(:,1:2),x_rand) ,[],1); 
    x_nearest= RRTree(I,1:2);
    theta=atan2(x_rand(1)-x_nearest(1),x_rand(2)-x_nearest(2));
    %获取采样点x_new
    x_new = double(int32(x_nearest(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~(feasiblePoint(ceil(x_new),map) && feasiblePoint(floor(x_new),map) && ... 
            feasiblePoint([ceil(x_new(1)) floor(x_new(2))],map) && feasiblePoint([floor(x_new(1)) ceil(x_new(2))],map))  
        continue; 
    end  %判断该点是否碰撞
%     if distanceCost(x_new,goal)<disTh, pathFound=true;break; end % goal reached
   
    X_near = NearestVertices(x_new,RRTree);
    if  size(X_near)==[0,0]
        [~, I1]=min(distanceCost(RRTree(:,1:2),x_new) ,[],1); 
        X_near = [RRTree(I1,1:3),I1];
    end
    Ls=GetSortedList(x_new,X_near,RRTree);
    x_min=ChooseBestParent(Ls,map);
   
    %从 X_near 选出x_min作为父节点
    if ~size(x_min)==[0 0]
%         theta=atan2(x_new(1)-x_min(1),x_new(2)-x_min(2)); 
%         x_new = double(int32(x_min(1:2) + stepsize * [sin(theta)  cos(theta)]));
        RRTree=[RRTree;[x_new(1:2),x_min(4)]];
         %插入新的节点
        RRTree=RewireVertices(x_min,x_new,Ls,RRTree,map);
        if Cost(RRTree,RRTree(end,:))+distanceCost(RRTree(end,1:2),goal)<cost_best && distanceCost(x_new,goal)<disTh
            cost_best=Cost(RRTree,RRTree(end,:))+distanceCost(RRTree(end,1:2),goal);
            tree1=RRTree;
            pl(i)=cost_best;
        end
    else
        continue;
    end
end
t=toc;

% if display && pathFound
%     disp('planing done!');
%     waitforbuttonpress;
%     if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end
%     hold on;
%     scatter(10,10,50,[1 0 0],'filled');
%     scatter(490,490,50,[0 1 0],'filled');
%     path=[source];
%     for i=2:length(RRTree)
%         son=RRTree(i,1:3);
%         waitbar(i/length(RRTree));
%         scatter(son(2),son(1),5,[0 0 1],'filled');
%         father=RRTree(son(3),1:2);
%         line([son(2);father(2)],[son(1);father(1)],'color',[240 128 128]/250);
%         counter=counter+1;M(counter)=getframe;
%     end
% end

if ~pathFound, error('no path found. maximum attempts reached'); end
path=[tree1(end,1:2);goal];
prev=tree1(end,3); 

while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end
line(path(:,2),path(:,1),'LineWidth',2.5,'Color',[0 1 0]);hold on;
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', t,pathLength); 
% imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(path(:,2),path(:,1),'Color','green','LineWidth',1.5);