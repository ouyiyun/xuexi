%   计算引力
%   调用此函数前先将goal转换到车体坐标系下
function att=attract(theta, goal_trans)
obj_k=goal_trans(2)/goal_trans(1);
obj_theta=rad2deg(atan(obj_k));
if obj_theta<0
    obj_theta=obj_theta+180;
end
att=cos(deg2rad(theta-obj_theta));
end