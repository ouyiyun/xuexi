function inLimits = checkLimits(envmap, state)
% Check if the state is within the environment limits. Return true if
% within limits, else return false
% INPUTS:
% envmap - map of environment
% state  - (x,y)
% OUTPUT:
% inLimits - True if within limits, false if not

if (state(1) < 1 || state(1) > size(envmap, 2) || ...
    state(2) < 1 || state(2) > size(envmap, 1))
    inLimits = false;
else
    inLimits = true;
end

end