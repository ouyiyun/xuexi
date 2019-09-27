%   �������ڵ�״̬pose����һ����yaw��nav_angle�������һ����״̬
%   pose=[x y yaw v]
function new_pose=cal_pose(pose, nav_angle, best_pf)
vmax=0.2;
theta=deg2rad(nav_angle);
dt=0.01;
v=vmax*best_pf/100;
new_pose(1)=v*cos(theta)*dt+pose(1);
new_pose(2)=v*sin(theta)*dt+pose(2);
new_pose(3)=nav_angle;
new_pose(4)=v;
end