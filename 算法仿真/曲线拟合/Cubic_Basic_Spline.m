function Cubic_Basic_Spline(x,y)
if any(size(x) ~= size(y)) || size(x,2) ~= 1
   error('inputs x and y must be column vectors of equal length');
end

n=length(x);

% ²åÖµ
main=[];
upper=[];
lower=[];
for i=1:n
    if i==1
        main=[main;6];
    elseif i==n
        main=[main;6];
    else
        main=[main;4];
    end
end

for i=1:n-2
    lower=[lower;1];
    upper=[upper;1];
end

lower=[lower;0;0];
upper=[0;0;upper];

T = spdiags([lower main upper], [-1 0 1], n, n);

px = T\(6*x);
py = T\(6*y);

px=[(2*px(1)-px(2));px;(2*px(end)-px(n-1))];
py=[(2*py(1)-py(2));py;(2*py(end)-py(n-1))];
plot(px,py,'go--');

for i=1:n-1
   t=linspace(0,1,100);
   xx=(px(i)+4*px(i+1)+px(i+2))/6-(px(i)-px(i+2)).*t/2+(px(i)-2*px(i+1)+px(i+2)).*t.^2/2 ...
       -(px(i)-3*px(i+1)+3*px(i+2)-px(i+3)).*t.^3/6;
   yy=(py(i)+4*py(i+1)+py(i+2))/6-(py(i)-py(i+2)).*t/2+(py(i)-2*py(i+1)+py(i+2)).*t.^2/2 ...
       -(py(i)-3*py(i+1)+3*py(i+2)-py(i+3)).*t.^3/6;
   plot(xx,yy,'g-');
end


% ÄâºÏ
% x = [(2*x(1)-x(2));x;(2*x(end)-x(n-1))];
% y = [(2*y(1)-y(2));y;(2*y(end)-y(n-1))];
% 
%  for i=1:n-1
%    t=linspace(0,1,100);
%    xx=(x(i)+4*x(i+1)+x(i+2))/6-0.5*(x(i)-x(i+2)).*t+0.5*(x(i)-2*x(i+1)+x(i+2)).*t.^2 ...
%        -(x(i)-3*x(i+1)+3*x(i+2)-x(i+3)).*t.^3/6;
%    yy=(y(i)+4*y(i+1)+y(i+2))/6-(y(i)-y(i+2)).*t/2+(y(i)-2*y(i+1)+y(i+2)).*t.^2/2 ...
%        -(y(i)-3*y(i+1)+3*y(i+2)-y(i+3)).*t.^3/6;
%    plot(xx,yy,'g-');    
% end
end