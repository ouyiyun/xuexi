function[fpath, cost] = rrtstar(envmap, start, goal, deltaStep)
% Implements the RRT-Star motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap  - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start   - Robot start [x,y] = [col,row]
% goal    - Robot goal [x,y] = [col,row]
% deltaStep - (Optional) Approx. number of cells by which to extend the graph in
%             direction of the nearest-neigbour on the graph (Default: 10)
% OUTPUTS:
% fpath   - Final collision-free path (N x 2, each row is [x,y])
% cost    - Cost of the final path (sum of squared distances between nodes on the path)

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
title('RRT-Star planning');
xlim([1,size(envmap,2)]);
ylim([1,size(envmap,1)]);

%% READ -> STUDENT TODO:
% Implement RRT to find a collision free path. Note that the edges also
% have to be collision-free, not just the nodes.
%   a) Sample states at random 99% of time, 1% sample goal state
%   b) Extend the graph by "deltaStep" steps from the nearest
%   neighbour of the sample in the direction of the sample. If sample is
%   within deltaStep distance, use it directly
%   c) Check within a ball around "xnew" to find min cost state to "xnew".
%   Set that as the parent
%   d) Check if any other state in the ball around "xnew" has a better path
%   through "xnew". If so, rewire the graph.
%   c) Approx. Collision checking for the (approx.) straight line path
%   between two states can be done using the function "collcheckstline"
%   d) Run till you find a state on the graph rounding which you get the
%   goal state
%   e) Display the progression of the graph generation for RRTStar (figure(fg))
ct = 0;
while(1)
    % TODO: Run till goal is reached
    % Sample random state (sample goal with 1% prob)
    % Get Nearest Neighbour on graph (out of collision, within limits)
    % Extend graph towards sample by deltaStep to get xnew (check for path collision)
    % Add xnew to graph, find states within ball of xnew. Find least cost path to xnew through states in the ball
    % Check in ball to find any other states that have lesser cost when reached via xnew. If so, add that edge and rewire the graph
    % Do some display (intermittently)
    % Check for reaching goal & repeat
    
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

% Draw a final time before exiting
figure(fg);
plot(fpath(:,1), fpath(:,2), 'k.-', 'Linewidth',2);
drawnow;

end