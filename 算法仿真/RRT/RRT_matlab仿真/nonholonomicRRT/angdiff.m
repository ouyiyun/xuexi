function d = angdiff(th1, th2)
% INPUT: 
% th1 - (N x 1) vector or a scalar
% th2 - scalar
% OUTPUT:
% d   - (N x 1) vector or a scalar angular difference mod to be within -180
%       to +180 degrees
    d = th1-th2;
    d = mod(d+pi, 2*pi) - pi;
    d = d*(180/pi); % convert to degrees
end