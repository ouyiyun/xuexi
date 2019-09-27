function [newPoint, extendFaile, I1]=exTree(RRTree, sample, stepsize, map, disTh)
%I1 newPoint的父节点

    extendFaile=false;
    %找到最近点,拓展RRTree
    [~, I1]=min(distanceCost(RRTree(:,1:2), sample), [], 1);
    %生成新节点
    newPoint=extendPoint(RRTree(I1,1:2), sample, stepsize);
    %新节点是否有效（1、是否是已存在的节点；2、q_new与q_near之间是否可连）
    [A, ~]=min(distanceCost(RRTree(:,1:2), newPoint), [], 1);
    if A <= disTh || ~checkPath(RRTree(I1,1:2), newPoint, map)
        extendFaile=true;
    end
end