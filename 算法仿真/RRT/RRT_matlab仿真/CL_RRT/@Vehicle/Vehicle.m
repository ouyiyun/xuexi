classdef Vehicle < handle
    %VEHICLE ��ü
    %   ��������κ��� ������ ������� ������ �����ϱ� ���� �𵨸��� �ⱸ���� ������� ��� �Լ��� ����
    
    properties
                                               %  ������                  ����                                           ����
                                               % ===============================================================================================================
        Pos                                    %  ��ġ                  �迭(4X1)             [x��ǥ(meter),y��ǥ(meter),heading(radian),���� ����(radian)]
        Vel                                    %  �ӵ�                  �迭(4X1)          [x�� �ӵ�(m/s),y�� �ӵ�(m/s),heading ���ӵ�(rad/s),���� ���ӵ�(rad/s)]
        Acc                                    %  ���ӵ�                �迭(4X1)     [x�� ���ӵ�(m/s^2),y�� ���ӵ�(m/s^2),heading �����ӵ�(rad/s^2),���� �����ӵ�(rad/s^2)]
        Speed                                  %  �ӷ�                    ����
        AccMag                                 %  ���ӵ� ũ��             ����
        
        deltaMax                               %  �ִ����Ⱒ               ����          radian, ������ ������ �ִ� ��, || {���Ⱒ��} || =< {�ִ����Ⱒ}
        deltaMaxDot                            %  �ִ����Ⱒ�ӵ�           ����          rad/s, ������ ������ �ִ� ���ӵ�, || {���Ⱒ�ӵ�} || =< {�ִ����Ⱒ�ӵ�}
        steerTime                              %  ���Ⱒ�ӵ� ���ް��      ����          ���Ⱒ�ӵ� ��ȭ�� ���� 1�� lag �����Լ��� ���. {���Ⱒ�ӵ���ȭ}=1/{���Ⱒ�ӵ����ް��} * ({�������Ⱒ�ӵ�}-{�������Ⱒ�ӵ�})
        charaVel                               %  ���̵彽������           ����          m^2/s^2, ���̵彽��(side-slip) ���� ����. G_ss = 1 / (1 + ({�ӵ�}/{���̵彽������})^2)
        
        accTime                                %  ���ӵ� ���ް��          ����          ���ӵ� ��ȭ�� ���� 1�� lag �����Լ��� ���. {���ӵ���ȭ}=1/{���ӵ����ް��} * ({�������ӵ�}-{�������ӵ�})
        accMin                                 %  �ּ� ���ӵ�              ����          m/s^2, �ּ� ���ӵ� ������ �����ϴ� ���ӷ��� �ִ��� ���� �ǹ�
        accMax                                 %  �ִ� ���ӵ�              ����          m/s^2, �ִ� ���ӵ� ������ �����ϴ� ���ӷ��� �ִ��� ���� �ǹ� 
        length                                 %  ����κ�������ձ���      ����          meter. ����κ��� �� ����� �� ���� ������ ����
        ratioLfw                               %  �������̺���             ����          ����κ��� �����ؾ� �� ���������� �Ÿ��� ������ձ��̷� ���� ��.
        
        speedAcc                               %  �ӵ���ɰ�꺯��(����)    ����          m/s^2, ����κ��� �ӵ���ɰ��� ����ϱ� ���Ͽ� ������ ���ӵ�����
        speedDec                               %  �ӵ���ɰ�꺯��(����)    ����          m/s^2, ����κ��� �ӵ���ɰ��� ����ϱ� ���Ͽ� ������ ���ӵ�����
        speedMax                               %  �ִ�ӵ�                 ����          m/s, ����κ��� �ӵ���� �ִ밪. ���� �׷����� ���� Ȯ��
        coastVelTime                           %  �ӵ�������ӽð�          ����          second, �ִ�ӵ��� �ƴ� �ӵ��� ��ɰ��� ���ӵǴ� �ð� 
        
        decAlphaZero                           %  ���Ӹ𵨰��(0����)       ����          meter, ���� �극��ũ�� ��뿡 ���� �̵��Ÿ��� ����� ���׽�(polynomial)�� 0���� ���
        decAlphaOne                            %  ���Ӹ𵨰��(1����)       ����          second, ���� �극��ũ�� ��뿡 ���� �̵��Ÿ��� ����� ���׽�(polynomial)�� 1���� ���
        decAlphaTwo                            %  ���Ӹ𵨰��(2����)       ����          s^2, ���� �극��ũ�� ��뿡 ���� �̵��Ÿ��� ����� ���׽�(polynomial)�� 2���� ���
        
        constThrottle                          %  �����߷������̵���     �迭(3X1)       �����ؾ� �� �߷¿� �ʿ��� �����̵��� ����ϱ� ���� ���׽��� ���
        constThrVel                            %  �߷´��ӵ������Լ���� �迭(2X1)       �߷¿� ���� �ӵ��� �����Լ� ��� 
        speedModel                             %  �߷�-�ӵ� �����Լ�      �����Լ�(TF)     �߷¿� ���� �ӵ��� �����Լ�
        speedModelProfile                      %  �߷¼ӵ������Լ����     �迭(2Xn)       ������ �־��� �ӵ��� TF ��, ����� ��޵Ǵ� ���� ������ �׵�� STEP ��ȣ �Է½� ����� ǥ���� ����
        
        hisPos                                 %  ��ġ����            �迭(4X{iterSim})            [x��ǥ(meter),y��ǥ(meter),heading(radian),���� ����(radian)]
        hisVel                                 %  �ӵ�����            �迭(4X{iterSim})         [x�� �ӵ�(m/s),y�� �ӵ�(m/s),heading ���ӵ�(rad/s),���� ���ӵ�(rad/s)]
        hisAcc                                 %  ���ӵ�����          �迭(4X{iterSim})    [x�� ���ӵ�(m/s^2),y�� ���ӵ�(m/s^2),heading �����ӵ�(rad/s^2),���� �����ӵ�(rad/s^2)]
        hisSpeed                               %  �ӷ�����            �迭({iterSim})
        hisAccMag                              %  ���ӵ� ũ�� ����     �迭({iterSim})
        

    end
    
    methods
        function setInitialVehicleState(veh,pos,vel,acc)                                               % ����κ� ���� ����(��ġ, �ӵ�, ���ӵ�) �ʱ�ȭ
           veh.Pos=pos;
           veh.Vel=vel;
           veh.Acc=acc;
           veh.Speed = norm(vel(1:2));
           veh.AccMag = norm(acc(1:2));
           
           veh.hisPos(1,1:4) = pos';
           veh.hisVel(1,1:4) = vel';
           veh.hisAcc(1,1:4) = acc';
           veh.hisSpeed(1) = veh.Speed;
           veh.hisAccMag(1) = veh.AccMag;
        end
        
        function dispVehicle(veh)                                                                      % ����κ��� �׷����� ǥ��
            figure(1)
            plot(veh.Pos(1),veh.Pos(2),'go');
        end
        
        function setTalos(veh)                                                                         % TALOS (MIT ���������ڵ��� ��Ī) ����
            veh.deltaMax = 0.5435;
            veh.deltaMaxDot = 0.3294;
            veh.steerTime = 0.05;
            veh.accTime = 0.3;
            veh.accMin = -6.0;
            veh.accMax = 1.8;
            veh.length = 2.885;
            veh.ratioLfw = 0.4;
            veh.charaVel = 20;
            
            veh.speedAcc = 1;                    % m/s^2
            veh.speedDec = 2.5;                  % m/s^2
            veh.speedMax = 11;                   % m/s (from result graph in the paper.)
            veh.coastVelTime = 3;                % s (not determined in the paper.)
            
            veh.decAlphaZero = -0.5347;          % m
            veh.decAlphaOne = 1.2344;            % s
            veh.decAlphaTwo = -0.0252;           % s^2
            
            veh.constThrottle = [0.1013 0.5788 49.1208];  % [c1 c2 c3] : Kn(v) = c1*v^2 + c2*v + c3
            veh.constThrVel = [12 1];                     % [tau_v 1] : V(s) = Kn(V)*U(s) / (tau_v(s)+1)
        end
        
        function setVechicleSpeedTF(veh)                                                                % �߷¿� ���� ����κ��� �ӵ��� ���ϱ� ���� �����Լ��� ����
            veh.speedModel = tf(1,veh.constThrVel);
            % step ��ȣ�� ���� ������ �����ͷ� �޴´�. �� ���� �����̵��� ���Ͽ� �ӵ��������� Ȱ���Ѵ�.
            [veh.speedModelProfile(1,:),veh.speedModelProfile(2,:)] = step(veh.speedModel);
        end
        
        function CalculateSpeed(veh,ctrl,sim)                                                           % �ӵ� ��� �Լ�
            % �ӵ��� ������Ʈ �ϸ� �ӵ������� ���� �ٲ�Ƿ� ���� �ӵ����庯���� ������Ʈ �� �ӵ��� �Ҵ��Ѵ�.
            veh.hisSpeed(sim.iterSim)=veh.Speed;
            
            % PI ��� ���Ͽ� �߷��� �����̵溯���� �����ȴ�.
            %  ����, �ʱ���� ��������� �ӵ� ������ �ջ��Ѵ�.
            %  �� ���� �̿��� I �����̵�� ���ϰ�, ������ ���¸� �̿��� P �����̵�� ���Ѵ�.
            ctrl.sumVehicleSpeed = ctrl.sumVehicleSpeed+(ctrl.refVel-veh.Speed)*sim.deltaT;
            ThrottleInput=ctrl.Kp*(ctrl.refVel-veh.Speed)+ctrl.Ki*ctrl.sumVehicleSpeed;
           
            % Kn(v) (�ӵ��� ���� �߷� �����̵氪)�� ����Ѵ�.
            Kn = veh.constThrottle(1)*veh.Speed^2+veh.constThrottle(2)*veh.Speed+veh.constThrottle(3);

            % ����κ��� ���� �ӵ��� �����ö󽺺�ȯ �������� ǥ���� ����ν� ����Ѵ�. ( V(t) = a*(1-e^(t/a'))
            veh.Speed = veh.Speed+Kn*ThrottleInput*(1-1/exp(sim.deltaT/veh.constThrVel(1)));
        end
        
        function CalculateState(veh,ctrl,sim)                                                           % ����κ� ���� ��� �Լ�
            % ���� ����κ��� ��ġ�� �ӵ�, ���ӵ��� ���庯���� �Ҵ��Ѵ�.
            veh.hisPos(sim.iterSim,:) = veh.Pos;
            veh.hisVel(sim.iterSim,:) = veh.Vel;
            veh.hisAcc(sim.iterSim,:) = veh.Acc;
            
            % ���� ����κ��� ���Ⱒ�ӵ��� �����Ͽ� ������Ʈ�Ѵ�.
            veh.Vel(4) = min(max((ctrl.refSteer-veh.hisPos(sim.iterSim,4))/veh.steerTime,-veh.deltaMaxDot),veh.deltaMaxDot);
            % ����κ��� ���Ⱒ���� �����Ͽ� ������Ʈ�Ѵ�.
            veh.Pos(4) = veh.hisPos(sim.iterSim,4)+veh.Vel(4)*sim.deltaT;
            % ����κ��� ������ �ӵ��� x,y�� �ӵ��� ���Ⱒ�ӵ��� ���Ѵ�. 
            %  x,y�� �ӵ��� �ӷ��� ���� ���, ���Ⱒ�ӵ��� ���̵彽���� ����Ͽ� ���Ѵ�. 
            %  {���Ⱒ�ӵ�} = {�ӵ�}/{������հŸ�} * tan({���Ⱒ��}) * {���̵彽�����(G_ss)}
            veh.Vel(1:3)... 
            = [veh.hisSpeed(sim.iterSim)*cos(veh.hisPos(sim.iterSim,3)),...
                veh.hisSpeed(sim.iterSim)*sin(veh.hisPos(sim.iterSim,3)),...
                veh.hisSpeed(sim.iterSim)*tan(veh.Pos(4))/veh.length*1/(1+(veh.hisSpeed(sim.iterSim)/veh.charaVel)^2)];
            % ����κ� �ӵ��� �̿��Ͽ� ����κ��� ��ġ�� ���Ѵ�.
            veh.Pos = veh.hisPos(sim.iterSim,:)+veh.Vel*sim.deltaT;
            % ����κ� �ӵ��� �̿��Ͽ� ����κ��� ���ӵ��� ���Ѵ�.
            veh.Acc = (veh.Vel-veh.hisVel(sim.iterSim,:))/sim.deltaT;
            % ����κ��� ���ӷ��� ����κ� ���ӵ��� ũ���̴�.
            veh.AccMag = norm(veh.Acc(1:2));
        end
       
        function PlotVehicleTrajectory(veh)                                                 % ����κ��� ������ �׷����� ǥ���ϴ� �Լ�
            figure(1)
            plot(veh.hisPos(:,1),veh.hisPos(:,2),'b.','MarkerSize',6);
            hold on
        end
    end
    
end

