%   ����ת��
%   x'=xcos��+ysin��-acos��-bsin��
%   y'=-xsin��+ycos��+asin��-bcos��
%   ��ƫ����Ϊ�������y��
function ob=transform(a, b, yaw, point)
ob=[];
theta=deg2rad(yaw-90);
for i=1:size(point,1)
    x=point(i,1);
    y=point(i,2);
    x_=x*cos(theta)+y*sin(theta)-a*cos(theta)-b*sin(theta);
    y_=-x*sin(theta)+y*cos(theta)+a*sin(theta)-b*cos(theta);
    ob=[ob;x_ y_];
end
end