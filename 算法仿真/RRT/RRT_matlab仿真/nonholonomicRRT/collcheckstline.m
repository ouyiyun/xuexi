function [isFree, stlinepath, stlinecost] = collcheckstline(envmap, start, goal)
% Collision check all states on the straight line path from start to goal
% INPUTS:
% envmap - Environment map
% start  - Start state [x,y]
% goal   - Goal state [x,y]
% OUTPUTS:
% isFree - True if path is free of collisions, false otherwise
% stlinepath - Discretized straight line path between start & goal (N x 2)
%              (8-connected), each row is [x,y]
% stlinecost - Cost of straight line path from start to goal

% Check if the path to nextState from nearState is collision-free
[ptx, pty] = bresenham(round(start(1)), round(start(2)), round(goal(1)), round(goal(2)));
isFree = true;
start
goal
ptx
pty
for k = 1:numel(ptx) % Check end-points as well
    if(envmap(pty(k), ptx(k)))
        isFree = false;
        break;
    end
end
stlinepath = [ptx pty];
stlinecost = sum(sqrt(sum((stlinepath(1:end-1,:) - stlinepath(2:end,:)).^2,2)),1);

end