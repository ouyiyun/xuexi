function caa=ConcernedAngleArea2(theta)
if theta-6<45
    caa=[45 theta+6];
elseif theta+6>135
    caa=[theta-6 135];
else
    caa=[theta-6 theta+6];
end
end