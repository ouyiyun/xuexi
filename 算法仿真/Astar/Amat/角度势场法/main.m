%   Dsr=0.2
%   Dm=ra=2     raΪ�����״��ɨ��뾶 
clear all;
% close all;
clc;

pose=[0 0 90 0];    %pose=[x y yaw v]
goal=[2 2];
% obstacle=[5 5; 
%           4 2;  
%           5 4; 
%           5 6;  
%           5 9  
%           8 9  
%           7 9  
%           6 5  
%           6 3   
%           6 8  
%           6 7  
%           7 4  
%           9 8  
%           9 11  
%           9 6];  
obstacle=[     0     1.0000;
            0.1000    0.9000;
            0.2000    0.8000;
            0.3000    0.7000;
            0.4000    0.6000;
            0.5000    0.5000;
            0.6000    0.4000;
            0.7000    0.3000;
            0.8000    0.2000;
            0.9000    0.1000;
            1.0000         0];
% ģ��ʵ��Ľ��,��������Щ��  ;
result.pose=[0 0 90 0]; 
tic;

disp('angle potential field start!!')
plot(pose(1),pose(2),'*g');hold on;  
for i=1:5000
    ob_trans=transform(pose(1), pose(2), pose(3), obstacle);
    goal_trans=transform(pose(1), pose(2), pose(3), goal);
    
    %   ���㵱ǰ����ѵ����Ƕ�
    [best_yaw,best_pf]=navigation_angle(ob_trans, goal_trans, pose(3));
    
    %   ����λ��
    pose=cal_pose(pose, best_yaw, best_pf);
    
    result.pose=[result.pose; pose];
    
    %   �ж��Ƿ񵽴�Ŀ���
    goal_dist=normest(pose(1:2)-goal);
    if goal_dist<0.01
        disp('Arrive Goal!!')
        break;
    end
    
    plot(result.pose(:,1),result.pose(:,2),'-g');hold on;  
    plot(goal(1),goal(2),'*r');hold on; 
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;  
    
end

disp('angle potential field end!!')
toc;
    