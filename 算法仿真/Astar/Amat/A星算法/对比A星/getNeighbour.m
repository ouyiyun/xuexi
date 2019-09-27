function outList=getNeighbour(currentPoint,n)
outList = [];
[x0,y0] = ind2sub([n,n],currentPoint.num);
Xmin=mLimit(x0-1,1,n);
Xmax=mLimit(x0+1,1,n);
Ymin=mLimit(y0-1,1,n);
Ymax=mLimit(y0+1,1,n);
x = Xmin:1:Xmax;
y = Ymin:1:Ymax;
[X,Y]=meshgrid(x,y);
i = 1;
for count = 1:numel(X)
    if X(count)==x0 && Y(count)==y0
        continue
    else
        outList=[outList,sub2ind([n,n],X(count),Y(count))];
        i=i+1;
    end
end
end