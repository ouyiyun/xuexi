function mydrawnow(starNum,goalNum,banList,closeList,openlist,n)
dbstop if error
x = 0:1:n;
y = 0:1:n;
[x,y]=meshgrid(x,y);
% [m,n]=size(x);
% [i,j]=ndgrid(1:m,1:n);
% ????1
dat = zeros(n,n);
[m,n]=size(dat);
dat(starNum)=0.5;
dat(goalNum)=0.8;
dat(banList)=0.2;
dat(closeList)=0.5;
dat(openlist)=0.9;
surf(x,y,zeros(size(x)),dat);
hold on;
view(2);
end