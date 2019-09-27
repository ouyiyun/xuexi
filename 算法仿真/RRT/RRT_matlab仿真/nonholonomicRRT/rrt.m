function[fpath, cost] = rrt(envmap, start, goal, deltaStep)
% Implements the RRT motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap    - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start     - Robot start [x,y] = [col,row]
% goal      - Robot goal [x,y] = [col,row]
% deltaStep - (Optional) Approx. number of cells by which to extend the graph in
%             direction of the nearest-neigbour on the graph (Default: 10)
% OUTPUTS:
% fpath     - Final collision-free path (N x 2, each row is [x,y])
% cost      - Cost of the final path (sum of squared distances between nodes on the path)

if nargin < 3
    error('Need to pass in map, start and goal');
end

if nargin < 4
    deltaStep = 10;
end
fprintf('Deltastep = %f \n',deltaStep);

% Figure
fg = figure(101); hold on;
imagesc(envmap); 
t1 = text(start(1), start(2), 'S'); set(t1,'Color','r','Fontsize',15);
t2 = text(goal(1), goal(2), 'G'); set(t2,'Color','g','Fontsize',15);
title('RRT planning');
xlim([1,size(envmap,2)]);
ylim([1,size(envmap,1)]);

%% READ -> STUDENT TODO:
% Implement RRT to find a collision free path. Note that the edges also
% have to be collision-free, not just the nodes.
%   a) Sample states at random 99% of time, with 1% probability sample the goal state
%   b) Extend the graph by "deltaStep" steps from the nearest
%   neighbour of the sample in the direction of the sample (along the
%   straight line). If sample is within deltaStep distance, use it directly
%   c) Approx. Collision checking for the (approx.) straight line path
%   between two states can be done using the function "collcheckstline".
%   This also returns the states along the straight line path.
%   d) A state is an obstacle if:
%       envmap(state(2), state(1)) = 1
%      A state is free if:
%       envmap(state(2), state(1)) = 0
%   e) Run till you find a state on the graph rounding which you get the
%   goal state
%   f) Display the progression of the graph generation for RRT (figure fg).
%   You can use the "plot" command in MATLAB to plot the edges of the graph
%   as you build it.
%   g) Use checkLimits.m to check for limits for the point robot. Round-off
% states for collision checking.
%   h) Once you have reached the goal state, you will need to backtrack to
%   find the path from start to goal.
% 
% (x,y) = (col,row) in MATLAB sense
% 
ct = 0;
RRTree=double([start -1]); 
stepsize=1; % size of each step of the RRT
disTh=5; % nodes closer than this threshold are taken as almost the same
failedAttempts=0;
while(1)
    % TODO: Run till goal is reached
    % Sample random state (sample goal with 1% prob)
    % Get Nearest Neighbour on graph (out of collision, within limits)
    % Extend graph towards sample by deltaStep to get xnew (check for path collision)
    % Add xnew to graph, do some display (intermittently)
    % Check for reaching goal & repeat
    
    if rand < 0.5,
        sample = rand(1,2) .* size(envmap); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    closestNode = RRTree(I(1),1:2);
    theta = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    
    if distanceCost(sample,goal)<1
       pathFound=true;
       break;
    end
    
    if ~collcheckstline(envmap,closestNode, newPoint) % if extension of closest node in tree to the new point is feasible
        %failedAttempts=failedAttempts+1;
        continue;
    end
    
    [A, I2]=min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh, failedAttempts=failedAttempts+1;continue; end 
    RRTree = [RRTree;newPoint I(1)]; % add node
    
    % Display intermittently - assumes that student plots the graph
    if ~mod(ct,200)
        figure(fg);
        drawnow;
    end
    
    % Increment counter
    ct = ct+1; 
end

% TODO: Backtrack to find the path between start & goal as well as the cost of the path
% You need to set variables fpath & cost
counter=0;
% Draw a final time before exiting
if pathFound 
    line([closestNode(2);goal(2)],[closestNode(1);goal(1)]);
    counter=counter+1;M(counter)=getframe;
end

%path_smooth = smooth(map, path, vertices, delta);
display=1;

if ~pathFound, error('no path found. maximum attempts reached'); end
path=[goal];
prev=I(1);
while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
imshow(envmap);%rectangle('position',[1 1 672 459],'edgecolor','k');
line(path(:,2),path(:,1));

fpath=0;
cost=0;

end