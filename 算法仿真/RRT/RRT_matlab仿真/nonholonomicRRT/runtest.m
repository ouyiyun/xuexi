function[fpath, cost] = runtest(mapfile, start, goal, seed, plannertype, varargin)
% Run a motion planning test on the given map for a point robot
% INPUTS:
% mapfile     - Path to the map of the environment (0 - free, 1 - obstacle)
% robotStart  - Start position of the robot ([x,y] = [col,row] MATLAB style)
% targetStart - Goal position of the robot ([x,y] = [col,row] MATLAB style)
% seed        - Deterministic seed
% plannertype - Type of the planner ('astar','rrt','rrtstar' - default: 'astar')
% varargin    - Variable arguments
%   For 'astar'   planner, you should pass in 'epsilon' for weighting the heuristics
%   For 'rrt'     planner, you should pass in 'deltastep', the number of cells to extend the graph by in each iteration
%   For 'rrtstar' planner, you can pass in what you want since it's extra-credit :P
% OUTPUTS:
% path        - Final collision free planned path (N x 2, where N = path length & each row is of the form [x,y])
% cost        - Cost of the final path
rng(seed); % Seed the random number generator
if nargin < 4
    error('Need to pass in mapfile, start, goal points and seed');
end

if nargin < 5
    plannertype = 'astar';
end

% Load the map
envmap = load(mapfile);
fg = figure(100); hold on;
imagesc(envmap);
title('Map of the environment');
t1 = text(start(1), start(2), 'S'); set(t1,'Color','r','Fontsize',15);
t2 = text(goal(1), goal(2), 'G'); set(t2,'Color','g','Fontsize',15);
xlim([1,size(envmap,2)]);
ylim([1,size(envmap,1)]);

% Check that the start & goal are within bounds
if(~checkLimits(envmap, start) || ~checkLimits(envmap,goal))
    error('Start and Goal state must be within the map limits');
end

% Run the planner and get results
if strcmp(plannertype,'astar')
    [fpath, cost] = astar(envmap, start, goal, varargin{:}); % VARIABLE ARGUMENTS ARE PASSED IN
elseif strcmp(plannertype,'rrt')
    [fpath, cost] = rrt(envmap, start, goal, varargin{:}); % VARIABLE ARGUMENTS ARE PASSED IN
elseif strcmp(plannertype,'rrtstar') % EXTRA-CREDIT
    [fpath, cost] = rrtstar(envmap, start, goal, varargin{:});
else
    error('Unknown planner type!');
end

% Check that the path is within bounds/collision free
for k = 1:size(fpath,1)
    pt = round(fpath(k,:)); % Round off the pt in case it is not an int
    if ~checkLimits(envmap, pt)
        error('State (%f, %f) on final path is out of bounds!',pt(1), pt(2));
    end
    if envmap(pt(2), pt(1))
        error('State (%f, %f) on final path is in collision!',pt(1), pt(2));
    end
end

% Display path
figure(fg);
plot(fpath(:,1), fpath(:,2), 'k.-', 'Linewidth',2);
fprintf('Final cost of the path: %f \n', cost);

end