%function[fpath] = nonholonomicrrt(datafile, start, goal, seed, ...
%                                  deltastep, numcontrolsamples,...
%                                  maxlinearvel, maxsteerangle)
%('nonholrrtdata.mat', [100,40,pi], [150,350,0], 100, 5)
% Run a RRT planning test on the given map for a car-like robot which is
% represented as a circle to make collision checking efficient.
% Car control is [linearvel, angularvel]
% INPUTS:
% datafile          - Obstacle and robot data (obstacle map & inflated map, robot radius)
% robotStart        - Start position of the robot - center of car & angle ([x,y,theta] - car robot)
% targetStart       - Goal position of the robot - center of car & angle ([x,y,theta] - car robot)
% seed              - Deterministic seed
% deltaStep         - (Optional) Approx. number of cells by which to extend the graph in
%                     direction of the nearest-neigbour on the graph 
%                     (Default: 5)            
% numcontrolsamples - (Optional) Number of control samples for finding a feasible path
%                     that grows the tree towards the sampled state from its NN
%                     (Default: 25)
% maxlinearvel      - (Optional) Maximum linear velocity for the car (control limit)
%                     (Default: 20)
% maxsteerangle     - (Optional) Maximum steering angle for the car (control limit)
%                     (Default: 0.6)
% OUTPUT:
% fpath             - Final collision free planned path (N x 3, where N = path length & each row is of the form [x,y,theta])
clear all
close all
datafile='nonholrrtdata.mat';
deltastep=5;
start=[100,40,pi];
goal=[150,350,0];
seed=100;
rng(seed); % Seed the random number generator

% if nargin < 4
%     error('Need to pass in datafile, start, goal points and seed');
% end

% Optional items
% if nargin < 5; deltastep = 5; end % 5 step propagation
% if nargin < 6; numcontrolsamples = 25; end % 25 control samples
% if nargin < 7; maxlinearvel = 20; end % Max linear velocity (20m/s)
% if nargin < 8; maxsteerangle = 0.6; end % Max steering angle limits (+-0.6 rad)

% Get the data
%%%% DO NOT MODIFY THIS VARIABLE
data = load(datafile);

% Display map, start and goal positions
fg = figure(100); hold on;
image(255 - data.envmap*255); colormap gray;
plotCarBot(data, start, 'b');
plotCarBot(data, goal, 'g');
xlim([1 data.mapsize(2)]);
ylim([1 data.mapsize(1)]);
axis square;
title('Map for non-holonomic RRT');

% Check that the start & goal are within bounds
if(checkLimitViolation_carBot(data,start) || checkLimitViolation_carBot(data,goal))
    error('Robot start and goal state must be within the map limits');
end

% Check that the start & goal are not in collision
if(checkCollision_carBot(data,start) || checkCollision_carBot(data,goal))
    error('Robot start and goal state must not be in collision');
end

%% READ -> STUDENT TODO: Run planner and get results
%% PLEASE READ!!!!!!!!!!!!!!!!!!
% Implement RRT for this non-holonomic car system. The state of the vehicle
% is 3D: [x,y,theta] where theta is the bearing of the vehicle. The system
% is constrained such that it can reach only a limited set of positions and
% orientations from its current state. To compute a feasible path for this
% system, the steps are as follows:
%
%  a) Sample a random pose (x,y,theta) (with probability 5% sample
%  the goal)
%
%  b) Find the nearest neighbour on the graph to the sample. You need to
%  implement the distance metric for this. This will be the function in
%  "distance.m". 
%
%  c) Extend the graph from the nearest neighbour in the direction of the
%  random sample. For this non-holonomic system, we do this by sampling
%  controls and forward propagating them through the system's equations of
%  motion. You need to sample "numcontrolsamples" samples and forward
%  propagate them using the function:
%  [xnew, isValid, rollout] = simulate_carBot(data, xnear, xrand, deltastep, 
%   linearvel, steerangl)
%  This function returns the new state to be added to the graph "xnew", the
%  actual rollout "rollout" and if this rollout is within limits & out of
%  collision "isValid". You need to choose the "xnew" that is closest to
%  the random sample out of all the control samples (and is valid). How you sample
%  controls is up to you, but a uniform random sampling within the control
%  limits (-maxlinearvel, maxlinearvel), (-maxsteerangle, maxsteerangle) works
%
%  d) All distance measurements between pairs of states has to be made
%  using the function "distance.m" which has to be implemented by you.
%
%  e) Given the best "xnew", you can add this to the graph and repeat
%
%  f) Run this until you reach a state that is at a reasonable distance to
%  the goal. In practice, I've seen that a state with an (x,y) distance of
%  7.5 cells & an angular distance of 2.5 degrees to the goal is reachable.

%  g) As before, please display the graph as it is grown. You can use the
%  "plot" command in MATLAB to display the graph edges.
%  
% There are a few other helper functions that you can use:
%     For checking samples for limit violation use the function:
%        checkLimitViolation_carBot(data, pose) where data is given to you
%     This function returns true if there is a limit violation
%     For checking for collisions, use the function:
%        checkCollision_carBot(data,pose)
%     This function returns true if there is a collision.
% NOTE: The collision checker assumes that the query pose is within limits,
% so please call the "checkLimitViolation_carBot" function before checking
% for collision.
% NOTE: The variable "data" is pre-defined. Please do not modify its
% contents.

% ct = 0;
% tempV=double([start -1]);
% RRTree=[tempV b_];
%RRTree=[node];
RRTree=double([start -1 0 0 0]);
stepsize=1; % size of each step of the RRT
disTh=5; % nodes closer than this threshold are taken as almost the same
failedAttempts=0;
numcontrolsamples=25;
x_new=[start];
I_=0;
while(1)
    
    if rand > 0.5
        xrand(1:2)= rand(1,2) .* size(data.envmap); % random sample
    else
        xrand(1:2)= goal(1:2); % sample taken as goal to bias tree generation to goal
    end
    xrand(3)=2*pi*(rand*2-1);
    
    % Get NN on graph (update function in "distance.m")
    [A, I]=min(distanceCost(RRTree(:,1:2),xrand(1:2)) ,[],1); % find closest as per the function in the metric node to the sample
    xnear = RRTree(I(1),1:3);
    
    maxlinearvel = 60;
    maxsteerangle = 0.6;
    distance2xrand=1000;
    dist=0;
    rollout_=[];
    lvel=0;
    lang=0;
    % 对速度进行采样
    for i = 1:numcontrolsamples
        %sample control
        %linearvel= maxlinearvel*(rand*2-1);
        linearvel= maxlinearvel*rand;
        steerangl= maxsteerangle*(rand*2-1);
        %simulate motion using "simulate_carBot" to get xnew & rollout
        [xnew, isValid, rollout,time] = simulate_carBot(data, xnear, xrand, deltastep, linearvel, steerangl);
        if isValid
            dist=distanceCost(xnew,xnear);
            if dist<distance2xrand
                x_new=xnew;
                rollout_=rollout;
                lvel=linearvel;
                lang=steerangl;
                ltime=time;
%                 plot(rollout(:,1),rollout(:,2));
%                 hold on
%                 drawnow
            end
        end
    end
    
%     if sqrt((x_new(1)-107.6)^2+(x_new(2)-76.9)^2)<3
%        pause;
%     end
    
    RRTree = [RRTree;x_new I(1) lvel lang ltime]; % add node
    I_=I(1);

    if size(rollout_,1)>1
        plot(rollout_(:,1),rollout_(:,2),'g','LineWidth',3,'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5])
        hold on
        drawnow
    end
    
    if distanceCost(x_new(1:2),goal)<10
        break; % goal reached
    end

end
pause(5);
path=[];
prev=I_;
while prev>0
    path=[RRTree(prev,1:3) RRTree(prev,5:7);path];
    prev=RRTree(prev,4);
end
line(path(:,1),path(:,2));
%%      
x = path(1,1:3);  
for i=1:size(path,1)
    dt1 = 0.1; % Step by 0.1 seconds
    rollout1 = x;
    L1 = 7.5; % Car length
    linearvel1=path(i,4);
    steerangl1=path(i,5);
    for i=1:path(i,6)
        x(1) = x(1) + linearvel1 * cos(x(3)) * dt1;
        x(2) = x(2) + linearvel1 * sin(x(3)) * dt1;
        x(3) = x(3) + (linearvel1/L1) * tan(steerangl1) * dt1;
        rollout1 = [rollout1; x];   % maintain history
    end
    if size(rollout1,1)>1

        plot(rollout1(:,1),rollout1(:,2),'r','LineWidth',3,'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5])
        hold on
        drawnow
    end
    for i=1:size(rollout1,1)
        plotCarBot(data, rollout1(i,:), 'k'); % Do not plot start/goal
        pause(0.05);
    end

end



%% Nothing to do here - Display path
% Check that the path is within bounds/collision free

% for k = 1:size(fpath,1)
%     pt = fpath(k,:);
%     if checkLimitViolation_carBot(data, pt)
%         error('State (%f, %f, %f) on final path is out of bounds!',pt(1), pt(2), pt(3));
%     end
%     if checkCollision_carBot(data, pt)
%         error('State (%f, %f, %f) on final path is in collision!',pt(1), pt(2), pt(3));
%     end
% end

% Display path
% figure(fg);
% for k = 2:size(fpath,1)-1
%     plotCarBot(data, fpath(k,:), 'k'); % Do not plot start/goal
%     pause(0.05);
% end



%% NOTHING TO DO FROM THIS PART ONWARDS
% simulate_carBot(data, xnear, xrand, deltastep, linearvel, steerangl)
function [xnew, isValid, rollout,time] = simulate_carBot(data, xnear, xrand, deltastep, linearvel, steerangl)
% Simulates a given control from the nearest state on the graph to the
% random sample.
% INPUTS:
% data      - Map and Robot data
% xnear     - Nearest point on the current graph to the random sample
% xrand     - Random sample
% deltastep - Number of steps to forward propagate (grow the graph)
% linearvel - Linear velocity of the car (control)
% steerangl - Steering angle of the car (control)
% OUTPUTS:
% xnew      - Potential new point to add to the graph for the current control
% isValid   - Flag to indicate if the rollout is valid (within limits & no
%             collisions). If isValid is false, you should discard this xnew/rollout.
% rollout   - Actual path found by the system from xnear->xnew

    % Set vehicle constants
    dt = 0.1; % Step by 0.1 seconds
    L = 7.5; % Car length
    
    % Simulate forward from xnear using the controls (linearvel, steerangl)
    % to generate the rollout
    x = xnear;
    rollout = x;
    for i=1:deltastep
        x(1) = x(1) + linearvel * cos(x(3)) * dt;
        x(2) = x(2) + linearvel * sin(x(3)) * dt;
        x(3) = x(3) + (linearvel/L) * tan(steerangl) * dt;
        rollout = [rollout; x];   % maintain history
    end
    
    % Find the closest point to xrand on the rollout
    % This is xnew. Discard the rest of the rollout
    dst = distance(rollout, xrand);
    [A, I]=min(distanceCost(rollout(:,1:2),xrand(1:2)) ,[],1);   
    %[~, id] = min(dst, [], 1);
    xnew = rollout(I(1), :); 
    time=I(1)-1;
    % rollout(id+1:end,:) = []; % Do not need this part now
    % Check for collision on the path:
    isValid = true;
    for i=1:size(rollout,1)
        % For each point, check if it is in violation of the limits or if
        % it is in collision. If either, return isValid = false
        if(checkLimitViolation_carBot(data, rollout(i,:)) || ...
           checkCollision_carBot(data, rollout(i,:)))
            isValid = false;
            break;
        end
    end
    
    % In case the rollout is not valid, set xnew to empty
    if ~isValid
        xnew = [];
        return;
    end
    
end

function isColliding = checkCollision_carBot(data, pose)
% Returns true if robot is colliding with obstacles, false otherwise
% Assumes that the state is within feasible limits
% INPUT:
% data - Map and robot data
% pose - pose of robot [x,y,theta] => center of robot & angle
% OUTPUT: 
% isColliding - True if robot is in collision, false otherwise
    if data.inflatedmap(round(pose(2)), round(pose(1)))
        isColliding = true;
    else
        isColliding = false;
    end
end

function outOfLimits = checkLimitViolation_carBot(data, pose)
% Returns true if robot violates limits, false otherwise
% INPUT:
% data - Map and robot data
% pose - pose of robot [x,y,theta] => center of robot & angle
% OUTPUT: 
% inLimits - True if robot violates limits, false otherwise
    outOfLimits = false;
    if ((pose(1) <= data.robotradius) ||...
        (pose(1) >= data.mapsize(2) - data.robotradius) || ...
        (pose(2) <= data.robotradius) || ...
        (pose(2) >= data.mapsize(1) - data.robotradius))
            outOfLimits = true;
    end
end

function H = plotcircle(center, radius, NOP, cl)
    THETA=linspace(0,2*pi,NOP);
    RHO=ones(1,NOP)*radius;
    [X,Y] = pol2cart(THETA,RHO);
    X=X+center(1);
    Y=Y+center(2);
    H = fill(X,Y,cl);
end

function plotCarBot(data, pose, cl)
% Plots the car robot as a box
% INPUT:
% data - Map and robot data
% pose - pose of robot [x,y,theta] => center of robot & angle
% cl   - color of the box outline
    if nargin < 3; cl = 'w'; end
    
    % Plot a circle for the car
    H = plotcircle(pose(1:2), data.robotradius, 50, cl);
    set(H, 'Facecolor', 'w', 'FaceAlpha', 0.6);
    
    % Now plot a line for the direction of the car
    ed = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))] * [data.robotradius*1.5; 0];
    plot([pose(1), pose(1)+ed(1)], [pose(2), pose(2)+ed(2)], 'b', 'Linewidth', 3);
end

function d = angdiff(th1, th2)
% Returns angle difference in -180 to 180
    d = th1-th2;
    d = mod(d+pi, 2*pi) - pi;
    d = d*(180/pi); % convert to degrees
end