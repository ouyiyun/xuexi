clear all;
close all;
clc;

x=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15]';
y=[1.2 0.6 0.0 1.5 3.8 5.0 4 3.0 2 1 0 2 3 4 5 4.5]';
% filepath=fullfile('E:','ros_bag','test.bag');
% bag=rosbag(filepath);
% message=select(bag,'MessageType','nav_msgs/Path');
% data=readMessages(message);

% x=[];
% y=[];
% 
% for i=1:47
%     x=[x;data{1,1}.Poses(i,1).Pose.Position.X];
%     y=[y;data{1,1}.Poses(i,1).Pose.Position.Y];
% end

plot(x,y,'ro--');
hold on

% [s0,s1,s2,s3]=cubic_spline1(x,y);
% plot_cubic_spline1(x,s0,s1,s2,s3);

id=fitting_points(x,y);
num=length(id);
x_size=length(x);
h_id=[];
if isempty(id)
    disp('this curve do not fitting');
else
    for i=1:num-1
        if id(i+1)-id(i)>=3
            h_id=[h_id;id(i)+1;id(i+1)-1];
        end
    end
    
    if isempty(h_id)
        start_id=id(1)-1;
        end_id=id(end)+1;
        [s0,s1,s2,s3]=cubic_spline1(x(start_id:end_id),y(start_id:end_id));
        plot_cubic_spline1(x(start_id:end_id),s0,s1,s2,s3);
%         Cubic_Basic_Spline(x(start_id:end_id),y(start_id:end_id));
    else
        h_id=[id(1)-1;h_id;id(end)+1];
        n=length(h_id)/2;
        for j=1:n
            start_id=h_id(2*j-1);
            if start_id==1
                k1=0;
            else
                k1=(y(start_id)-y(start_id-1))/(x(start_id)-x(start_id-1));
            end
            end_id=h_id(2*j);
            if end_id==x_size
                k2=0;
            else
                k2=(y(end_id)-y(end_id+1))/(x(end_id)-x(end_id+1));
            end
            [s0,s1,s2,s3]=cubic_spline(k1,k2,x(start_id:end_id),y(start_id:end_id));
            plot_cubic_spline(x(start_id:end_id),s0,s1,s2,s3);
            [s0,s1,s2,s3]=cubic_spline1(x(start_id:end_id),y(start_id:end_id));
            plot_cubic_spline1(x(start_id:end_id),s0,s1,s2,s3);
        end
    end
end


% 固定边界
function [s0,s1,s2,s3]=cubic_spline(k1,k2,x,y) %x,y表示待插值点序列
 
if any(size(x) ~= size(y)) || size(x,2) ~= 1
   error('inputs x and y must be column vectors of equal length');
end
 
n = length(x);
 
h = x(2:n) - x(1:n-1);
d = (y(2:n) - y(1:n-1))./h;
% for i=1:n-1
%     if d(i)==inf
%         d(i)=1000;
%     end
% end
A=d(1)-k1;
B=k2-d(end);
 
lower = [h;0];
main  = 2*(h(1:end-1) + h(2:end));
main  = [2*h(1);main;2*h(end)];
upper = [0;h];

% 构建稀疏矩阵
T = spdiags([lower main upper], [-1 0 1], n, n);
rhs =6*[A;(d(2:end)-d(1:end-1));B];

m = T\rhs;
 
% Use natural boundary conditions where second derivative
% is zero at the endpoints
 
s0 = y(1:n-1);
s1 = d - h.*m(1:n-1)./3-h.*m(2:n)./6;
s2 = m(1:n-1)/2;
s3 =(m(2:end)-m(1:end-1))./(6*h);
end

% 自由边界
function [s0,s1,s2,s3]=cubic_spline1(x,y) %x,y表示待插值点序列
 
if any(size(x) ~= size(y)) || size(x,2) ~= 1
   error('inputs x and y must be column vectors of equal length');
end
 
n = length(x);
 
h = x(2:n) - x(1:n-1);
d = (y(2:n) - y(1:n-1))./h;
 
lower = h(2:end);
main  = 2*(h(1:end-1) + h(2:end));
upper = h(1:end-1);

% 构建稀疏矩阵
T = spdiags([lower main upper], [-1 0 1], n-2, n-2);
rhs = 6*(d(2:end)-d(1:end-1));

m = T\rhs;
 
% Use natural boundary conditions where second derivative
% is zero at the endpoints
 
m = [ 0; m; 0];
 
s0 = y(1:n-1);
s1 = d - h.*m(1:n-1)./3-h.*m(2:n)./6;
s2 = m(1:n-1)/2;
s3 =(m(2:end)-m(1:end-1))./(6*h);
end

function plot_cubic_spline(x,s0,s1,s2,s3)
 
n = length(x);
 
for i=1:n-1
   xx = linspace(x(i),x(i+1),30);
   xi = repmat(x(i),1,30);
   yy = s0(i) + s1(i)*(xx-xi) + ... 
     s2(i)*(xx-xi).^2 + s3(i)*(xx - xi).^3;
   plot(xx,yy,'b')
end
end

function plot_cubic_spline1(x,s0,s1,s2,s3)
 
n = length(x);
 
for i=1:n-1
   xx = linspace(x(i),x(i+1),30);
   xi = repmat(x(i),1,30);
   yy = s0(i) + s1(i)*(xx-xi) + ... 
     s2(i)*(xx-xi).^2 + s3(i)*(xx - xi).^3;
   plot(xx,yy,'k')
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
