classdef Control < handle
    %CONTROL ��ü
    %   ����κ��� ������ �� Ⱦ���� ����� ���õ� ���� �� �Լ��� ���Ե�
    
    properties
                                                       %  ������                  ����                                           ����
                                                       % ===============================================================================================================
        Kp                                             % �߷��Է� P-gain          ����             �߷� �Է°��� ����� ���� �ӵ��� ���� P-gain. Kp * ({�����ӵ�}-{����ӵ�}) 
        Ki                                             % �߷��Է� I-gain          ����             �߷� �Է°��� ����� ���� �ӵ��� ���� I-gain. Ki * int({�����ӵ�}-{�ӵ�} fr. t=0 to t=current) 
        Kd                                             % �߷��Է� D-gain          ����             �߷� �Է°��� ����� ���� �ӵ��� ���� D-gain. Kd * d/dt({�����ӵ�}-{����ӵ�}) 
        
        sumVehicleSpeed                                % �߷��Է� ���⺯��         ����             m/s, �߷� �Է°��� �����̵� �� I-gain�� ���ϱ� ���Ͽ� �ӵ������� ���� �� ����ϴ� ����
        
        lfw                                            % �����Ÿ�                 ����             meter, ������ġ ��� �������� �־�� �� ���Ÿ��� ��Ÿ����, �ӵ��� ���� ���Ӻ���
        refLfw                                         % �����������������۰Ÿ�   ����            meter, �� �������κ��� ������(look-ahead line)�� �����ϴ� �Ÿ� 
        posLfw                                         % ��������ġ              �迭(2X1)             [x��ǥ(meter), y��ǥ(meter)], ������(look-ahead point) ��ġ
        lfwMin                                         % �ּ������Ÿ�              ����            meter, �����Ÿ��� ����κ��� �ӵ��� ���� �Լ��� ǥ���Ǵµ�, �� �� �ּ����� �Ÿ�
        lfwMax                                         % �ִ������Ÿ�              ����            meter, �����Ÿ��� ����κ��� �ӵ��� ���� �Լ��� ǥ���Ǵµ�, �� �� �ִ����� �Ÿ�
        lfwSlope                                       % �����Ÿ���ȭ��            ����            second, �����Ÿ��� ��ȭ�� �ӵ��� ���� �Լ��� ��Ÿ��, {�����Ÿ�} = {�����Ÿ���ȭ��}X{�����ӵ�}
        
        refVel                                         % �����ӵ�                  ����            m/s, ����κ��� �����ؾ� �� �ӵ�
        refCoastVel                                    % ��������ӵ�              ����            m/s, ����κ��� ����/�����Ͽ� �����ϰ� �����ϴ� ���¿����� �ӵ�
        refSteer                                       % �������Ⱒ��              ����            radian, ����κ��� �����ؾ� �� ���Ⱒ��
        refSteerEta                                    % ����������                ����            radian, ��μ��� ������ ������ ����
        
        hisLfw                                         % �����Ÿ�����        �迭(1X{iterSim})     ���ǽ��� ��������� ���� �����Ÿ� ���� ����
        hisPosLfw                                      % ��������ġ����      �迭(1X{iterSim})     ���ǽ��� ��������� ���� ��������ġ ���� ����
        hisRefVel                                      % �����ӵ�����        �迭(1X{iterSim})     ���ǽ��� ��������� ���� �����ӵ� ���� ����
        hisRefCoastVel                                 % ��������ӵ�����     �迭(1X{iterSim})     ���ǽ��� ��������� ���� ��������ӵ� ���� ����
        hisRefSteer                                    % �������Ⱒ������     �迭(1X{iterSim})     ���ǽ��� ��������� ���� �������Ⱒ�� ���� ����
        hisRefSteerEta                                 % ��������������       �迭(1X{iterSim})     ���ǽ��� ��������� ���� ���������� ���� ����
    end
    
    methods
        function setLookAheadDistance(ctrl,veh)                                                         % �����Ÿ� �� ������ ����
            ctrl.refLfw = veh.ratioLfw*veh.length;
            ctrl.posLfw = veh.hisPos(1,1:2);
        end
        
        function setControlTalos(ctrl)                                                                  % TALOS (MIT ���������ڵ��� ��Ī) ���� �� �ʱ�ȭ
            ctrl.lfwMin = 3;
            ctrl.lfwMax = 12;
            ctrl.lfwSlope = 2.24;
            ctrl.refVel = 0;
            ctrl.sumVehicleSpeed = 0;
        end
        
        function setPID(ctrl,p,i,d)                                                                     % �߷��Է°� ���� PID �����̵� ����
            ctrl.Kp = p;
            ctrl.Ki = i;
            ctrl.Kd = d;
        end
        
        function SteeringControl(ctrl,env,veh,sim)                                                      % ��������
            % �����Ÿ��� �����Ѵ�.
            %  �����ӵ��� 1.34m/s �̸��̸� �ּ������Ÿ�
            %  �����ӵ��� 1.34m/s �̻� 5.36m/s �̸��̸� {�����Ÿ���ȭ��}*{�����ӵ�}
            %  �����ӵ��� 5.36m/s �̻��̸� �ִ������Ÿ�
            ctrl.lfw=(ctrl.refVel<1.34)*ctrl.lfwMin+...
                (ctrl.refVel>=5.36)*ctrl.lfwMax+...
                ((ctrl.refVel>=1.34)&&(ctrl.refVel<5.36))*(ctrl.lfwSlope*ctrl.refVel);
            
            % ���Ⱒ�� ���ϱ� ���Ͽ� ������ ���� ���Ⱒ �������� ����Ѵ�.
            %  1) ����κ��� ��ġ���� ������������� ���� ������ ������ ������������ ��´�.            
            RefDirection = (env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2))/norm((env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2)));
            %  2) ����κ� ��ġ���� ������������� ���� ������ ���⿡�� ����κ��� ������ ���־� ����� ���Ⱒ�� ���Ѵ�.
            ctrl.refSteerEta=veh.Pos(3)-atan2(RefDirection(2),RefDirection(1));
            %  3) ������ ������ �������� �̿��Ͽ� �������Ⱒ�� ���Ѵ�.
            ctrl.refSteer = -atan2(veh.length*sin(ctrl.refSteerEta),ctrl.lfw/2+ctrl.refLfw*cos(ctrl.refSteerEta));
            %  4) �������Ⱒ�� ����κ��� ���Ⱒ ũ�⿡ ������ �����Ƿ� ���������� �����Ѵ�.
            ctrl.refSteer = min(max(ctrl.refSteer,-veh.deltaMax),veh.deltaMax);
        end
        
        
        
        function SpeedControl(ctrl,env,veh,sim)                                                      % �ӵ�����
            % ���� ��ġ���� ��������������� �Ÿ��� ����Ѵ�. 
            %  �� �� �����ؾ� �� ������� ����� �Ÿ��� ����Ѵ�.
            %  1) �������� ��ġ�� ���Ѵ�.
            % ctrl.posLfw=veh.Pos(1:2)+ctrl.lfw*[cos(ctrl.refSteerEta-veh.Pos(3)),-sin(ctrl.refSteerEta-veh.Pos(3))];
            % plot(ctrl.posLfw(1),ctrl.posLfw(2),'go');
            
            %  2) �������� ��ġ�������� ��������������� ���̸� ���Ѵ�.
            %  �� ���������� ������ġ-��������� ���� �Ÿ��� �Ͽ���. (�ܼ�ȭ)
            DistToGoal=norm(veh.Pos(1:2)-env.Waypt(sim.WayptIdx,1:2));
            %  3) ��������� ���ĺ��� ��������� ������ �Ÿ��� �����ش�.
            for iter = sim.WayptIdx : env.nWaypt-1
                DistToGoal=DistToGoal+env.distBtwWaypt(iter);
            end
            
            % ��������ӵ��� ���Ѵ�.
            %  �ӵ��� ���������� ���� ���¹ٿ� ���� ����-���-������ ����� ���̸�,
            %  �ӵ��� �������Ͽ� ���Ͽ� ����Ǵ� �̵��Ÿ��� ����Ѵ�.
            %  �ִ�ӷ����� ���ӵǾ� �����ϴٰ� ������ ����� �̵��Ÿ��� ũ�Ⱑ 
            %  ���� �� �Ÿ����� �۰� �ȴٸ�, ��������ӵ��� �ִ�ӷ��� �ȴ�.
            if (veh.speedMax^2/(2*veh.speedDec)+veh.decAlphaTwo*veh.speedMax^2+...
                    veh.speedMax*veh.coastVelTime+veh.decAlphaOne*veh.speedMax+veh.decAlphaZero+...
                    (veh.speedMax^2-veh.Speed^2)/(2*veh.speedAcc) < DistToGoal)
                ctrl.refCoastVel=veh.speedMax;
            %  ���� �׷��� �ʰ� ������ �Ÿ����� ũ�ų� ���ٸ�, ��������ӵ��� ����
            %  �����־�� �Ѵ�. �ִ� �ӷ����� ���� ���� �������� �ΰ� 2������ ��������
            %  Ǯ�� �ش�.
            else
                % 2�� �����Ŀ��� ���� ������ Ȱ���Ͽ� ���Ѵ�. 
                CoastVelEqnA=1/(2*veh.speedAcc)+1/(2*veh.speedDec)+veh.decAlphaTwo;
                CoastVelEqnB=veh.coastVelTime+veh.decAlphaOne;
                CoastVelEqnC=-veh.Speed^2/(2*veh.speedAcc)+veh.decAlphaZero-DistToGoal;
                ctrl.refCoastVel=(-CoastVelEqnB+sqrt(CoastVelEqnB^2-4*CoastVelEqnA*CoastVelEqnC))/(2*CoastVelEqnA);
            end
            
            % ��������ӵ��� ���� �ӵ����� ������ �ӵ��� �����ð�(time step)��ŭ ���ӵ� ���� �����ӵ��� �Ҵ��ϰ�,
            %  ������ ��������ӵ� ������ �Ҵ��ϸ�, �׷��� ������ �����ð� ��ŭ ���ӵ� ���� �����ӵ��� �Ҵ��Ѵ�.  
            ctrl.refVel = ctrl.refCoastVel;
            %ctrl.refVel=(veh.Speed<ctrl.refCoastVel)*(veh.Speed+veh.speedAcc*sim.deltaT)+(veh.Speed>=ctrl.refCoastVel)*(veh.Speed-veh.speedDec*sim.deltaT);
        end
        
        
        
        function setHistoryControlState(ctrl,sim)                                                       % ���� ���� ������ �����صд�.
            ctrl.hisLfw(sim.iterSim) = ctrl.lfw;
            ctrl.hisPosLfw(sim.iterSim,:) = ctrl.posLfw;
            ctrl.hisRefVel(sim.iterSim) = ctrl.refVel; 
            ctrl.hisRefCoastVel(sim.iterSim) = ctrl.refCoastVel;
            ctrl.hisRefSteer(sim.iterSim) = ctrl.refSteer;
            ctrl.hisRefSteerEta(sim.iterSim) = ctrl.refSteerEta;
        end
    end
    
end

