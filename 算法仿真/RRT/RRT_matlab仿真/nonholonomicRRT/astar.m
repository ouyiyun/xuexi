function[fpath, cost, displaymap] = astar(envmap, start, goal, epsilon)
% Implements the A-star motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap  - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start   - Robot start [x,y] = [col,row]
% goal    - Robot goal [x,y] = [col,row]
% epsilon - (Optional) Epsilon for epsilon a-star >= 1.0 (Default = 1.0)
% OUTPUTS:
% fpath      - Final collision-free path (N x 2, each row is [x,y])
% cost       - Cost of the final path
% displaymap - Map highlighted with closed & open states from a-star

if nargin < 3
    error('Need to pass in map, start and goal');
end

if nargin < 4
    epsilon = 1.0;
end
if epsilon < 0
    error('Epsilon has to be >= 0.0');
end
fprintf('Using epsilon = %f \n', epsilon);

% Have a map for display
displaymap = envmap;
fg = figure(101); hold on;
imgg = imagesc(displaymap);
t1 = text(start(1), start(2), 'S'); set(t1,'Color','r','Fontsize',15);
t2 = text(goal(1), goal(2), 'G'); set(t2,'Color','g','Fontsize',15);
title('A-star planning');
xlim([1,size(displaymap,2)]);
ylim([1,size(displaymap,1)]);

%% READ -> STUDENT TODO: 
% Implement A-star to find a collision free path. 
%  a) Use cost of g+epsilon*h, epsilon > 1 for epsilon a-star
%  where the heuristic (h) is Euclidean distance to the goal. Each transition
%  has a cost equal to the length of the transition:
%    cost of action (dx, dy) = sqrt(dx^2 + dy^2);
% (So actions that move left,right,up,down have cost = 1,
%  actions that move diagonally have cost = sqrt(2)). 
%  Use an 8-connected neighbourhood for A-star.
%  b) A state is an obstacle if:
%       envmap(state(2), state(1)) = 1
%     A state is free if:
%       envmap(state(2), state(1)) = 0
%  c) To check if a state is within limits, use the function "checkLimits"
%  d) Display the closed/open states for A-star as the search progresses.
%  You can easily do this by setting these states to have a specific value
%  in the variable "displaymap". For example, open states can have
%  value = 2 & closed states can have value = 3. Note that value of 0 means
%  free & 1 means obstacles.
%     displaymap(openState(2), openState(1)) = 2
%     displaymap(closedState(2), closedState(1)) = 3
%  e) Most implementations of A-star use a priority queue to sort the open
%  states before choosing the next one to expand. MATLAB does not have a 
%  native priority queue, but you can get around this by saving the
%  costs/corresponding states in two arrays and retrieving the min at each
%  step:
%       [minval, minid] = min(f_cost) % f_cost is a vector of costs
% minval is minimum cost value and minid is the element with the min cost.
%   f) Once you have reached the goal state, you will need to backtrack to
%   find the path from start to goal.
% (x,y) = (col,row) in MATLAB sense

ct = 0;
while(1)
    % TODO: Run till goal state is reached or there are no open states
    % If state is open -> displaymap(state(2), state(1)) = 2
    % If state is closed -> displaymap(state(2), state(1)) = 3
    
    % Display intermittently - assumes that displaymap has been modified
    % properly by the student
    if ~mod(ct,200)
        figure(fg);
        set(imgg, 'CData', displaymap);
        drawnow;
    end
    
    % Increment counter
    ct = ct+1;
end

% TODO: Backtrack to find shortest path & cost of the path
% You need to set variables fpath & cost

% Final display before exit
figure(fg);
set(imgg, 'CData', displaymap);
plot(fpath(:,1), fpath(:,2), 'k.-', 'Linewidth',2); % Display final path
drawnow;
fprintf('Number of closed states: %d \n', sum(sum(displaymap == 3)));

end