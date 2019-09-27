%计算关注域角度
%6为atan(0.2/2)的角度值
%0.2很重要 其为障碍物应该离的最小距离

function caa=ConcernedAngleArea(theta, ob_trans)
if theta-6<45
    caa=[45 theta+6];
elseif theta+6>135
    caa=[theta-6 135];
else
    caa=[theta-6 theta+6];
end

m=size(ob_trans,1);

for i=1:m
    ob_k=ob_trans(i,2)/ob_trans(i,1);
    if ob_k<0
        ob_theta=180-rad2deg(atan(ob_k));
    else
        ob_theta=rad2deg(atan(ob_k));
    end
    %计算障碍物的影响范围
    ob_dist=normest(ob_trans(i,:));
    ob_area=rad2deg(atan(0.4/ob_dist));
%     ob_area=rad2deg(asin(0.2/ob_dist));
    ob_caa=[ob_theta-ob_area ob_theta+ob_area];
    if ob_caa(1)<=caa(1) && (ob_caa(2)<=caa(2) && ob_caa(2)>=caa(1))
        caa(1)=ob_theta-ob_area;
    elseif (ob_caa(2)>=caa(2)) && (ob_caa(1)<=caa(2) && ob_caa(1)>=caa(1))
        caa(2)=ob_theta+ob_area;
    elseif ob_caa(1)<=caa(1) && ob_caa(2)>=caa(2)
        caa(1)=ob_theta-ob_area;
        caa(2)=ob_theta+ob_area;
    end
end
% if caa(1)<0
%     caa(1)=0;
% elseif caa(2)>180
%     caa(2)=180;
% end
end
        