classdef PathPlanner < handle
    % PathPlanner ��ü
    %   RRT ��ΰ�ȹ�� ���õ� ���� �� �Լ��� ���Ե�
    
    properties
                                                       %  ������                       ����                                           ����
                                                       % ===============================================================================================================
        NEcorner                                       % Environment ũ��(�ϵ�)        ����(1X2)             �۾������� ũ�� : �ϵ��� ���� 
        SWcorner                                       % Environment ũ��(����)        ����(1X2)             �۾������� ũ�� : ������ ���� 
        RandomPoint                                    % ���� ����                     ����                  �������� ������ sample
        EuclideanDistance                              % ��尣 �Ÿ�                   ����                  �������� ������ sample�� ������ ��尣�� �Ÿ�
        idx                                            % �ִܳ�� index                ����                  RandomPoint���� �ִܰŸ��� �ִ� ����� index 
        segmentLength                                  % RRT Ȯ�����                  ����                  RRT Ȯ�����
        cost                                           % ���� cost                     ����                  ���������� ���� �������� ������ cost 
        new_point                                      % ���� ������ ����Ʈ             �迭(1X2)             RandomPoint���� segmentLength���� Ȯ��� ���� point
        new_node                                       % ���� ������ ���               ����(1X5)             [new_point, 0, cost, idx]

    end
    
    
    
    
    
    
    methods
        function setLookAheadDistance(ctrl,veh)                                                         % �����Ÿ� �� ������ ����
            ctrl.refLfw = veh.ratioLfw*veh.length;
            ctrl.posLfw = veh.hisPos(1,1:2);
        end
        
        
        function setPID(ctrl,p,i,d)                                                                     % �߷��Է°� ���� PID �����̵� ����
            ctrl.Kp = p;
            ctrl.Ki = i;
            ctrl.Kd = d;
        end
        
        
        
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Draw Sample  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        function DrawSample(ctrl,env,veh,sim)                                                      % ��������
            p=rand;    
            if p < .2
                randomPoint = end_node(1:2);
            else
                randomPoint = [...
                    (env.bound(2)-env.bound(1))*rand,...
                    (env.bound(4)-env.bound(3))*rand]; 
            end
       
        end
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Node Selection  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        function NearestNeighbor(ctrl,env,veh,sim)                                                      % ��������
            % find leaf on node that is closest to randomPoint
            EuclideanDistance = tree(:,1:2)-ones(size(tree,1),1)*randomPoint;
            [dist,idx] = min(diag(tmp*tmp'));
            cost     = tree(idx,4) + segmentLength;
            new_point = (randomPoint-tree(idx,1:2));
            new_point = tree(idx,1:2)+new_point/norm(new_point)*segmentLength;
            new_node = [new_point, 0, cost, idx];       
        end        
        
        

    end
    
end

