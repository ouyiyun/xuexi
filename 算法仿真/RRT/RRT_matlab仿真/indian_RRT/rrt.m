% ? Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their 
% publication; and sharing the code publically is permitted without permission. 

% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using 
% Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available 
% at: http://rkala.in/codes.html

% 0 occupied 1 free
close all;
clear all;

map=im2bw(imread('maze.png')); % input map read from a bmp file. for new maps write the file name here
source=[50 60]; % source position in Y, X format
goal=[490 480]; % goal position in Y, X format
stepsize=40; % size of each step of the RRT
disTh=40; % nodes closer than this threshold are taken as almost the same
% 最大循环次数
maxFailedAttempts = 10000;
display=true; % display of RRT
[m, n]=size(map);

%%%%% parameters end here %%%%%
tic;
if ~feasiblePoint(source,map)
    error('source lies on an obstacle or outside map'); 
end

if ~feasiblePoint(goal,map)
    error('goal lies on an obstacle or outside map'); 
end
if display
    imshow(map);
    hold on
    plot(source(2),source(1),'r*');
    plot(goal(2),goal(1),'b*');
    % 绘制地图边框
    rectangle('position',[1 1 n-1 m-1],'edgecolor','b'); 
end

RRTree=double([source -1]); % RRT rooted at the source, representation node and parent index
failedAttempts=0;
counter=0;
pathFound=false;
while failedAttempts<=maxFailedAttempts  % loop to grow RRTs
    % 基于概率P的RRT
    if rand < 0.5
        sample=rand(1,2) .* size(map); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    
    % 原始RRT
%     sample=rand(1,2) .* size(map);
    % A为distanceCost矩阵的中每列的最小元素，I其所在的行数
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    closestNode = RRTree(I(1),1:2);
    % atan2(a,b)是4象限反正切
%     theta=atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
    % q_new
%     newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    % if extension of closest node in tree to the new point is feasible
    newPoint = double(findQNew(closestNode,sample,stepsize));
    if ~checkPath(closestNode(1:2), newPoint, map) 
        failedAttempts=failedAttempts+1;
        continue;
    end
    
    % 是否到达目标点附近
    if distanceCost(newPoint,goal)<disTh
        pathFound=true;
        break; 
    end % goal reached
    % check if new node is not already pre-existing in the tree
    [A, I2]=min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); 
    if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh
        failedAttempts=failedAttempts+1;
        continue; 
    end 
    RRTree=[RRTree;newPoint I(1)]; % add node
    failedAttempts=0;
    if display
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)]);
        plotpoint(RRTree,newPoint);
        counter=counter+1;
        M(counter)=getframe;
        %pause(1);
    end
end
if display && pathFound 
    line([closestNode(2);goal(2)],[closestNode(1);goal(1)]);
    counter=counter+1;M(counter)=getframe;
end
% if display 
%     disp('click/press any key');
%     waitforbuttonpress; 
% end
if ~pathFound, error('no path found. maximum attempts reached'); end
path=[goal];
prev=I(1);
while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end
pathLength=0;

pause(2);
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
imshow(map);rectangle('position',[1 1 n-1 m-1],'edgecolor','k');
line(path(:,2),path(:,1),'Marker','*');

function plotpoint(old_point, new_point)
[m, ~]=size(old_point);
for i=2:m
    plot(old_point(i,2),old_point(i,1),'k*');
end
plot(new_point(2),new_point(1),'g*');
end

function [q_new] = findQNew(q_near, q_rand, delta_q)

    v = double(q_rand - q_near);
    
    u = v / norm(v);
    
    q_new = int32(double(q_near) + delta_q * u);
    
end