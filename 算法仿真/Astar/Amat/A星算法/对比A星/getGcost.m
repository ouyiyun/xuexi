function cost = getGcost(currentPoint,n)
global point
    [xf,yf]=ind2sub([n,n],currentPoint.father);
    [xs,ys]=ind2sub([n,n],currentPoint.num);
    if xs==xf || yf==ys
        cost = point(currentPoint.father).Gcost+10*0.1;
    else
        cost = point(currentPoint.father).Gcost+14.14*0.1;
    end
end