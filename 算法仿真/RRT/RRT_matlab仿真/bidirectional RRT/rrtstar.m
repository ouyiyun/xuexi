clc 
clear all;
close all;

map=im2bw(imread('maze.png'));
source=[50 160];
goal=[490 490];
stepsize=40;
disTh=35;
r=80;
if ~feasiblePoint(source,map)
    error('起始点无效，请重新输入！1 free 0 occupy！');
end

if ~feasiblePoint(goal,map)
    error('目标点无效，请重新输入！1 free 0 occupy！');
end

counter=0;
imshow(map);
hold on
plot(source(2),source(1),'ro');
plot(goal(2),goal(1),'go');
counter=counter+1;M(counter)=getframe;

RRTree=[source 0 -1];
maxFaileNum=10000;
faileNum=0;
foundPath=false;
pause(10);
while faileNum<maxFaileNum || foundPath
    % 采样
    if rand < 0.5
        sample=double(int32(rand(1,2).*size(map)));
    else
        sample=goal;
    end
    
    %找x_nearest
    [~, I1]=min(distanceCost(RRTree(:,1:2),sample),[],1);
    nearestPoint=RRTree(I1,1:2);
    
    %生成newPoint
    newPoint=extendPoint(nearestPoint, sample, stepsize);
    %newPoint是否合法(1、已存在 2、newPoint与nearestPoint可连接)
    [A,~]=min(distanceCost(RRTree(:,1:2),newPoint),[],1);
    if A<=disTh || ~checkPath(nearestPoint, newPoint, map)
        faileNum=faileNum+1;
        continue
    end
    line([newPoint(2); nearestPoint(2)],[newPoint(1); nearestPoint(1)],'color','r');
    plot(newPoint(2),newPoint(1),'k*');
    % 找nearPoint
    nearPoint=findNearPoint(RRTree, newPoint, r);
    minCost=RRTree(I1,3)+distanceCost(nearestPoint, newPoint);
    xmin=nearestPoint;
    minId=I1;
    % Connect along a minimum-cost path
    [I2, ~]=size(nearPoint);
    for i=1:I2
        nowCost=nearPoint(i,3) + distanceCost(nearPoint(i,1:2),newPoint);
        if checkPath(nearPoint(i,1:2), newPoint, map) && nowCost < minCost
            xmin=nearPoint(i,1:2);
            minCost=nowCost;
            minId=nearPoint(i,4);
        end
    end
    RRTree=[RRTree; newPoint minCost minId];
    line([newPoint(2); xmin(2)],[newPoint(1); xmin(1)],'color','b');
    [n,~]=size(RRTree);
    %Rewire the tree
    for i=1:I2
        nowCost=minCost+distanceCost(newPoint,nearPoint(i,1:2));
        if checkPath(nearPoint(i,1:2),newPoint, map) && nowCost<nearPoint(i,3)
            line([newPoint(2); nearPoint(i,2)],[newPoint(1); nearPoint(i,1)],'color','b');
        end
    end
    counter=counter+1;M(counter)=getframe;
    %是否到达目标点附近
    if distanceCost(newPoint, goal) < disTh && checkPath(newPoint, goal, map)
        nowCost=RRTree(end, 3) + distanceCost(newPoint, goal);
        RRTree=[RRTree; goal nowCost n];
        foundPath=true;
        disp('已找到有效路径');
        break
    end
end

if faileNum>=maxFaileNum
    disp('无法找到有效路径');
end
path=[];
[prev,~]=size(RRTree);
n=prev;
pause(2);
if foundPath
    while prev>-1
        path=[path;RRTree(prev,1:2)];
        prev=RRTree(prev,4);
    end
end
imshow(map);
plot(path(:,2),path(:,1),'color','k');

function near=findNearPoint(Tree, point, th)
[m, ~]=size(Tree);
near=[];
for i=1:m
    if distanceCost(Tree(i,1:2), point) < th
        near=[near; Tree(i,1:3) i];
    end
end
end
 