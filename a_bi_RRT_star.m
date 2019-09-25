% ? Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 

% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
clear all;
close all;
% 原来是RRT，现在将其修改为 bi-RRT*
% 参考论文：
% 《Intelligent bidirectional rapidly-exploring random trees for optimal
% motion planning in complex cluttered environments》
% Algrithm 5: bi-RRT*
% 作者:钟 杰
% 2019/9/8

map=imbinarize(imread('map1.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % 【source position in Y, X format】注意这一点
goal=[490 490]; % 【goal position in Y, X format】注意这一点

stepsize=20; % size of each step of the RRT
disTh=20;    % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
display=true; % display of RRTree
N=5000;pl=zeros(1,N);
sigma_best=[source;goal.*100];
cost_best=1000;
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
for n=1:N
    %采样
    pl(n)=cost_best;
    flag1=0;flag2=0;
    n
    x_rand=rand(1,2) .* size(map); % random sample
    [~, I]=min(distanceCost(RRTree1(:,1:2),x_rand) ,[],1);
    x_nearest= RRTree1(I,1:2);
    theta=atan2(x_rand(1)-x_nearest(1),x_rand(2)-x_nearest(2));
    %获取采样点x_new
    x_new = double(int32(x_nearest(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~(feasiblePoint(ceil(x_new),map) && feasiblePoint(floor(x_new),map) && ... 
            feasiblePoint([ceil(x_new(1)) floor(x_new(2))],map) && feasiblePoint([floor(x_new(1)) ceil(x_new(2))],map))  
        continue; 
    end  %判断该点是否碰撞
    X_near = NearestVertices(x_new,RRTree1);
    if  size(X_near)==[0,0]
        [~, I1]=min(distanceCost(RRTree1(:,1:2),x_new) ,[],1); 
        X_near = [RRTree1(I1,1:3),I1];
        
    end
    Ls = GetSortedList(x_new,X_near,RRTree1);
    x_min = ChooseBestParent(Ls,map);

    if ~size(x_min)==[0 0]
        RRTree1=[RRTree1;[x_new(1:2),x_min(4)]];
        RRTree1=RewireVertices(x_min,x_new,Ls,RRTree1,map);
        flag1=1;
    end
 
    [~, I1]=min(distanceCost(RRTree2(:,1:2),x_new) ,[],1);
    x_connect=RRTree2(I1,1:2);
    %调整第二棵树
    [flag2,RRTree2]=ConnectGraphs(RRTree2,x_new,x_connect,stepsize,map);
    
    if flag1==1 && flag2==1
        if Cost(RRTree1,RRTree1(end,:))+Cost(RRTree2,RRTree2(end,:))<cost_best
            cost_best=Cost(RRTree1,RRTree1(end,:))+Cost(RRTree2,RRTree2(end,:));
            tree1=RRTree1;tree2=RRTree2;pl(n)=cost_best;
        end    
    end
    a=RRTree1;
    b=RRTree2;
    RRTree1=b;
    RRTree2=a;  
end

t=toc
% 绘制图
% for i=2:length(RRTree1)
%     i
%     son=RRTree1(i,1:3);
%     scatter(son(2),son(1),5,[0 0 255]/255,'filled');
%     father=RRTree1(son(3),1:2);
%     line([son(2);father(2)],[son(1);father(1)],'color',[250 128 114]/255);
%     counter=counter+1;M(counter)=getframe;
% end
% for i=2:length(RRTree2)
%     i
%     son=RRTree2(i,1:3);
%     scatter(son(2),son(1),5,[0 0 255]/255,'filled');
%     father=RRTree2(son(3),1:2);
%     line([son(2);father(2)],[son(1);father(1)],'color',[244 164 96]/255);
%     counter=counter+1;M(counter)=getframe;
% end
%% 绘制路径计算路径长度
path=[tree1(end,1:2)];
prev=tree1(end,3);
while prev>0
    path=[tree1(prev,1:2);path];
    prev=tree1(prev,3);
end

path1=[tree2(end,1:2)];
prev=tree2(end,3);
while prev>0
    path1=[tree2(prev,1:2);path1];
    prev=tree2(prev,3);
end  
line(path(:,2),path(:,1),'Color',[255 0 0]/255,'LineWidth',2);
line(path1(:,2),path1(:,1),'Color',[0 255 0]/255,'LineWidth',2);
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
for i=1:length(path1)-1, pathLength=pathLength+distanceCost(path1(i,1:2),path1(i+1,1:2)); end

fprintf('processing time=%d \nPath Length=%d \n\n', t,pathLength); 
































