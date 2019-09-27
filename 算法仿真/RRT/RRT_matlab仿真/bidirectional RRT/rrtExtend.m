% ? Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Bidirectional Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function [RRTree1,pathFound,extendFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map)
pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
failedAttempts=0;
while failedAttempts<=maxFailedAttempts
    % 基于一定概率向goal采样，增长的树不一样，其goal也不一样
    if rand < 0.5
        sample=rand(1,2) .* size(map); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    [~, I]=min( distanceCost(RRTree1(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    closestNode = RRTree1(I(1),:);
    newPoint=double(extendPoint(closestNode(:,1:2), sample, stepsize));
%     theta=atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
%     newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        continue;
    end
    [~, I2]=min( distanceCost(RRTree2(:,1:2),newPoint) ,[],1); % find closest in the second tree
    if distanceCost(RRTree2(I2(1),1:2),newPoint)<disTh % if both trees are connected
        if checkPath(RRTree2(I2(1),1:2),newPoint,map)
            pathFound=[newPoint I(1) I2(1)];
            extendFail=false;
            break;
        end
    end 
    [~, I3]=min( distanceCost(RRTree1(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree1(I3(1),1:2))<disTh
        failedAttempts=failedAttempts+1;
        continue;
    end 
    RRTree1=[RRTree1;newPoint I(1)];
    extendFail=false;
    break; % add node
end
end

function point=extendPoint(q_near, q_rand, stepsize)
p=q_rand-q_near;
v=p./norm(p);
point=int32(q_near+v*stepsize);
end