function [RRTree1, RRTree2, pathFound, treeFlag, num]=extextTree(RRTree1, RRTree2, goal, map, stepsize, disTh, maxK)
pathFound=[];
treeFlag=false;
count=0;
count1=0;
while count<=maxK
    if rand < 0.5
        sample=rand(1,2).*size(map);
    else
        sample=goal;
    end
    
    %拓展RRTree1
    [newPoint, extendFaile, I1]=exTree(RRTree1, sample, stepsize, map, disTh);
    if extendFaile
        count=count+1;
        continue
    end
        
    % newPoint是否在树二的附近,若在则两棵树连接
    [B, I2]=min(distanceCost(RRTree2(:,1:2), newPoint), [], 1);
    if B <= disTh && checkPath(RRTree2(I2,1:2), newPoint, map)
        RRTree1=[RRTree1;newPoint I1];
        pathFound=[newPoint I1 I2];
        disp('666');
        num=0;
        return
    end
    % 扩展RRTree2
    [nowPoint, extendFaile, I3]=exTree(RRTree2, newPoint, stepsize, map, disTh);
    if extendFaile
        count1=count1+1;
        if count1>=50
            RRTree1=[RRTree1;newPoint I1];
            treeFlag=false;
            num=0;
            return
        else
            continue
        end
    end
    
    % 树二生成的新节点与树1的新节点在一定阈值内可连接则返回pathFound
    if distanceCost(newPoint, nowPoint)<=disTh && checkPath(newPoint, nowPoint, map)
        RRTree1=[RRTree1; newPoint I1];
        RRTree2=[RRTree2; nowPoint I3];
        pathFound=[nowPoint I1 I3];
        disp('777');
        num=1;
        return
    else
        RRTree1=[RRTree1; newPoint I1];
        RRTree2=[RRTree2; nowPoint I3];
        num=1;
        return
    end
end

if count>maxK
    treeFlag=true;
    num=0;
end
end
