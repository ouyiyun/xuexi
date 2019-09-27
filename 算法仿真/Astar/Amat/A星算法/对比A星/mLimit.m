function [ y ] = mLimit( x,xmin,xmax )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if x>xmax
    y = xmax;
elseif x<xmin
    y = xmin;
else
    y = x;
end

end
