clear all;
clc;

filepath=fullfile('E:','ros_bag','test.bag');
bag=rosbag(filepath);
message=select(bag,'MessageType','nav_msgs/Path');
data=readMessages(message);

x=[];
y=[];

for i=1:46
    x=[x;data{1,1}.Poses(i,1).Pose.Position.X];
    y=[y;data{1,1}.Poses(i,1).Pose.Position.Y];
end

plot(x,y,'ko-');
hold on
id=fitting_points(x,y);

if isempty(id)
    disp('this curve do not fitting');
else
    for i=1:length(id)
        x_=[x(id(i)-1);x(id(i));x(id(i)+1);];
        y_=[y(id(i)-1);y(id(i));y(id(i)+1);];
        x_ = [(2*x_(1)-x_(2));x_;(2*x_(end)-x_(2))];
        y_ = [(2*y_(1)-y_(2));y_;(2*y_(end)-y_(2))];
        for j=1:2
           t=linspace(0,1,100);
           xx=(x_(j)+4*x_(j+1)+x_(j+2))/6-0.5*(x_(j)-x_(j+2)).*t+0.5*(x_(j)-2*x_(j+1)+x_(j+2)).*t.^2 ...
               -(x_(j)-3*x_(j+1)+3*x_(j+2)-x_(j+3)).*t.^3/6;
           yy=(y_(j)+4*y_(j+1)+y_(j+2))/6-0.5*(y_(j)-y_(j+2)).*t+0.5*(y_(j)-2*y_(j+1)+y_(j+2)).*t.^2 ...
               -(y_(j)-3*y_(j+1)+3*y_(j+2)-y_(j+3)).*t.^3/6;
           plot(xx,yy,'g-');    
        end
    end
end

function index=fitting_points(x,y)
    
    n=size(x);
    index=[];
    k_all=[];
    
    for i=2:n-1
        a=norm([x(i)-x(i-1) y(i)-y(i-1)],2);
        b=norm([x(i+1)-x(i) y(i+1)-y(i)],2);
        c=norm([x(i-1)-x(i+1) y(i-1)-y(i+1)],2);
        area=abs((x(i)-x(i-1))*(y(i+1)-y(i-1))-(x(i+1)-x(i-1))*(y(i)-y(i-1)))/2;
        k=4*area/(a*b*c);
        k_all=[k_all;k];
        if k>0.02
            index=[index;i];
        end
    end
end