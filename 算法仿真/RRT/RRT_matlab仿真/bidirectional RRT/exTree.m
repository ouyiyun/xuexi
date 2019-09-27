function [newPoint, extendFaile, I1]=exTree(RRTree, sample, stepsize, map, disTh)
%I1 newPoint�ĸ��ڵ�

    extendFaile=false;
    %�ҵ������,��չRRTree
    [~, I1]=min(distanceCost(RRTree(:,1:2), sample), [], 1);
    %�����½ڵ�
    newPoint=extendPoint(RRTree(I1,1:2), sample, stepsize);
    %�½ڵ��Ƿ���Ч��1���Ƿ����Ѵ��ڵĽڵ㣻2��q_new��q_near֮���Ƿ������
    [A, ~]=min(distanceCost(RRTree(:,1:2), newPoint), [], 1);
    if A <= disTh || ~checkPath(RRTree(I1,1:2), newPoint, map)
        extendFaile=true;
    end
end