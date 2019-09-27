classdef Environment < handle
    %ENVIRONMENT ��ü
    %   �ӹ��� �����ϴ� �ֺ�ȯ���� ������ ���� �Լ��� ���Ե�
    
    properties
                               %  ������                  ����                                           ����
                               % ===============================================================================================================
        nWaypt                 % ����� ����               ����                
        Waypt                  % ����� ��ġ          �迭({nWaypt}X2)                       [x��ǥ(meter), y��ǥ(meter)] 
        distBtwWaypt           % ����� ���� �Ÿ�     �迭({nWaypt-1}X1)       meter����, ���� ��������� �����ϱ����� ������ �Ÿ��� ����ϱ� ���Ͽ� ���
        
        bound                  % �ʵ� ũ��            �迭(4X1)             [x�� �ּҰ�(meter),x�� �ִ밪(meter),y�� �ּҰ�(meter),y�� �ִ밪(meter)]
    end
    
    methods
        
        function setBound(env,input)                                                % �ʵ� ũ�� ���� : input�� �޾Ƽ� �ʵ�ũ�⸦ �Ҵ�
            env.bound = input;
        end
        
        function setWptNum(env,input)                                               % ����� ���� ���� : input�� �޾Ƽ� ����� ������ �Ҵ�
            env.nWaypt = input;
        end
        
        function dispField(env)                                                     % �ʵ� ǥ�� : �Ҵ�� �ʵ�ũ�⸦ ����Ͽ� �ʵ� ��ü���� ǥ��
            figure(1)
            axis([env.bound(1),env.bound(2),env.bound(3),env.bound(4)]);
            xlabel('X-direction (meter)');
            ylabel('Y-direction (meter)');
            axis equal, grid on,  hold on;
%             axis tight,
        end
        
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  RRT Algorithm  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        
%         function setWpt(env)                                                        % ����� ���� : ���콺 Ŭ���� ���� ����� �Ҵ� : RRT �˰��� �߰��� ����
%             for iter=1:env.nWaypt
%                 %������� �����ϱ� ���Ͽ� ginput �Լ��� ����Ͽ�
%                 % �׷��� �󿡼� ����ڰ� ���Ǵ�� �Ҵ��� �ش�.
%                 env.Waypt(iter,1:2)=ginput(1);
%                 plot(env.Waypt(iter,1),env.Waypt(iter,2),'ro');
%             end
% 
%             %�Ҵ�� ����� ������ �Ÿ��� ���Ѵ�.
%             for iter=1:env.nWaypt-1
%                 env.distBtwWaypt(iter) = norm(env.Waypt(iter+1,1:2)-env.Waypt(iter,1:2));
%             end
%         end



%         function setWpt(env)                                                        % ����� ���� : ���콺 Ŭ���� ���� ����� �Ҵ� : RRT �˰��� �߰��� ����
%             env.Waypt = [0,0; 50 40;100 40;150 0;180 0];
%             env.nWaypt = size(env.Waypt,1);
%             plot(env.Waypt(:,1),env.Waypt(:,2),'r-o', 'LineWidth',3);
% 
%             %�Ҵ�� ����� ������ �Ÿ��� ���Ѵ�.
%             for iter=1:env.nWaypt-1
%                 env.distBtwWaypt(iter) = norm(env.Waypt(iter+1,1:2)-env.Waypt(iter,1:2));
%             end
%         end
        
           
        
        
        
        function setWpt(env,rrt)                                                        % ����� ���� : ���콺 Ŭ���� ���� ����� �Ҵ� : RRT �˰��� �߰��� ����
            env.Waypt = rrt.wpt(:,1:2);
            plot(env.Waypt(:,1),env.Waypt(:,2),'r-o', 'LineWidth',1);
            
            env.Waypt = rrt.wpt(2:end,1:2);
            env.nWaypt = size(env.Waypt,1);

            %�Ҵ�� ����� ������ �Ÿ��� ���Ѵ�.
            for iter=1:env.nWaypt-1
                env.distBtwWaypt(iter) = norm(env.Waypt(iter+1,1:2)-env.Waypt(iter,1:2));
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        


    end
    
end

