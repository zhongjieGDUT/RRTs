% ? Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 

% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
clear;
close all;
% 原来是RRT，现在将其修改为IB-RRT*
% 参考论文：
%《Intelligent bidirectional rapidly-exploring random trees for optimal
% motion planning in complex cluttered environments》
% Algrithm 5: RRT*
% 作者:钟 杰
% 2019/9/24

map=imbinarize(imread('map2.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % 【source position in Y, X format】注意这一点
goal=[490 490]; % 【goal position in Y, X format】注意这一点

stepsize=20; % size of each step of the RRT
disTh=20;    % nodes closer than this threshold are taken as almost the same
display=true; % display of RRTree
N=160000;pl=zeros(1,N);

%%%%% parameters end here %%%%%
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end
hold on;
scatter(10,10,50,[1 0 0],'filled');
scatter(490,490,50,[0 1 0],'filled');
RRTree1=double([source -1]); % First RRT rooted at the source, representation node and parent index
RRTree2=double([goal -1]); % Second RRT rooted at the goal, representation node and parent index
counter=0;
pathFound=true;
tic;
Connection=true;cost_best=1500;
for i=1:N
    %% 无碰采样
    pl(i)=cost_best;Connection=true;
    i
    x_rand=rand(1,2) .* size(map); % random sample
    if ~(feasiblePoint(ceil(x_rand),map) && feasiblePoint(floor(x_rand),map) && ... 
            feasiblePoint([ceil(x_rand(1)) floor(x_rand(2))],map) && feasiblePoint([floor(x_rand(1)) ceil(x_rand(2))],map))  
        continue; 
    end  %判断该点是否碰撞
    x_rand = double(int32(x_rand));
    %% 获取邻域节点
    X_near_1 = NearestVertices(x_rand,RRTree1);
    X_near_2 = NearestVertices(x_rand,RRTree2);
    if isempty(X_near_1)==1 && isempty(X_near_2)==1
        Connection = false;
        X_near_1 = NearestVertex(x_rand,RRTree1);
        X_near_2 = NearestVertex(x_rand,RRTree2);
    else
        if isempty(X_near_1)==1
            X_near_1 = NearestVertex(x_rand,RRTree1);
        end
        if isempty(X_near_2)==1
            X_near_2 = NearestVertex(x_rand,RRTree2);
        end 
    end
    
    Ls_1=GetSortedList(x_rand,X_near_1,RRTree1);
    Ls_2=GetSortedList(x_rand,X_near_2,RRTree2);
    %% get Best Tree Parent

    [x_min_1,cost1]= ChooseBestParentTree(Ls_1,map);
    [x_min_2,cost2] = ChooseBestParentTree(Ls_2,map);
    if cost1<=cost2 && ~isempty(x_min_1) %选择树1
        RRTree1=[RRTree1;[x_rand(1:2),x_min_1(4)]];
        RRTree1=RewireVertices(x_min_1,x_rand,Ls_1,RRTree1,map);
        if ~isempty(x_min_2) && Connection
            if Cost(RRTree1,RRTree1(end,:))+Cost(RRTree2,RRTree2(x_min_2(4),:))+distanceCost(x_rand,RRTree2(x_min_2(4),1:2))<cost_best
                cost_best=Cost(RRTree1,RRTree1(end,:))+Cost(RRTree2,RRTree2(x_min_2(4),:))+distanceCost(x_rand,RRTree2(x_min_2(4),1:2));
                tree1=RRTree1;
                tree2=RRTree2(1:x_min_2(4),:);
                pl(i)=cost_best;
            end    
        end
    else
        if cost1>cost2 && ~isempty(x_min_2)%选择树2
            RRTree2=[RRTree2;[x_rand(1:2),x_min_2(4)]];
            RRTree2=RewireVertices(x_min_2,x_rand,Ls_2,RRTree2,map);
            if ~isempty(x_min_1) && Connection
                if Cost(RRTree2,RRTree2(end,:))+Cost(RRTree1,RRTree1(x_min_1(4),:))+distanceCost(x_rand,RRTree1(x_min_1(4),1:2))<cost_best
                    cost_best=Cost(RRTree2,RRTree2(end,:))+Cost(RRTree1,RRTree1(x_min_1(4),:))+distanceCost(x_rand,RRTree1(x_min_1(4),1:2));
                    tree1=RRTree1(1:x_min_1(4),:);
                    tree2=RRTree2;
                    pl(i)=cost_best;
                end    
            end
        end
    end
end
t=toc;
%% 绘制路径计算路径长度
path=[tree1(end,1:2)];
prev=tree1(end,3);
while prev>0
    path=[RRTree1(prev,1:2);path];
    prev=RRTree1(prev,3);
end

path1=[tree2(end,1:2)];
prev=tree2(end,3);
while prev>0
    path1=[RRTree2(prev,1:2);path1];
    prev=RRTree2(prev,3);
end  
line(path(:,2),path(:,1),'Color',[255 0 0]/255,'LineWidth',2);
line(path1(:,2),path1(:,1),'Color',[0 0 255]/255,'LineWidth',2);
line([tree1(end,2) tree2(end,2)],[tree1(end,1) tree2(end,1)],'Color',[0 255 0]/255,'LineWidth',2);
pathLength=0;
if length(path)>1
    for i=1:length(path)-1
        pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); 
    end
end
if length(path1)>1
    for i=1:length(path1)-1
        pathLength=pathLength+distanceCost(path1(i,1:2),path1(i+1,1:2));
    end
end
pathLength=pathLength+distanceCost(tree1(end,1:2),tree2(end,1:2));
fprintf('processing time=%d \nPath Length=%d \n\n', t,pathLength); 























