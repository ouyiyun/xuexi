%   ������ѵ����Ƕ�
%   ����
%   �����ѵ����Ƕ�

function [best_yaw,best_pf]=navigation_angle(ob_trans, goal_trans, yaw)
best_angle=45;
best_pf=0;
for nav_angle=45:2:135
    caa=ConcernedAngleArea(nav_angle, ob_trans);
%     caa=ConcernedAngleArea2(nav_angle);
    rep=repulsion(caa, ob_trans);
    att=attract(nav_angle,goal_trans);
    pf=att/rep;
%     if pf>90
%         best_pf=pf;
%         best_angle=nav_angle;
%         break;
%     end
    if best_pf<pf
        best_pf=pf;
        best_angle=nav_angle;
    end
end
best_yaw=best_angle+yaw-90; %ͳһ����������ϵ
end