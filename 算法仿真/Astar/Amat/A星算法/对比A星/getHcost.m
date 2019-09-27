
function cost = getHcost(currentPoint,goalPoint,n)
    [xs,ys]=ind2sub([n,n],currentPoint.num);
    [xg,yg]=ind2sub([n,n],goalPoint.num);
    cost = abs(xs-xg)+abs(ys-yg);
end