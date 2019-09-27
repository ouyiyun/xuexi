function [RRTree, extendFaile, path, num]=conTree(RRTree1, RRTree, sample, stepsize, map, disTh)
% 找到最近的点
% RRTree需要扩展的树
extendFaile=false;
[~, I1]=min(distanceCost(RRTree(:,1:2), sample), [], 1);
newPoint=RRTree(I1,1:2);
oldPoint=RRTree(I1,1:2);
num=0;

while distanceCost(newPoint, sample) > disTh
    oldPoint=newPoint;
    newPoint=extendPoint(oldPoint, sample, stepsize);
    [A, ~]=min(distanceCost(RRTree(:,1:2), newPoint), [], 1);
    if checkPath(newPoint, oldPoint, map) && A > disTh
        [B, I]=min(distanceCost(RRTree1(:,1:2), newPoint), [], 1);
        if num==0
            [m,~]=size(RRTree);
            RRTree=[RRTree; newPoint I1];
            num=num+1;
            if B < disTh && checkPath(newPoint, RRTree1(I,1:2), map)
                path=[newPoint I m];
                return
            end
        else
            [m, ~]=size(RRTree);
            RRTree=[RRTree; newPoint m];
            num=num+1;
            if B < disTh && checkPath(newPoint, RRTree1(I,1:2), map)
                path=[newPoint I m];
                return
            end
        end
    else
        break
    end
end

if num==0
    extendFaile=true;
    path=[];
    return
end

if checkPath(newPoint, sample, map)
    [m, ~]=size(RRTree);
    path=[newPoint 0 m];
else
    path=[];
end

end