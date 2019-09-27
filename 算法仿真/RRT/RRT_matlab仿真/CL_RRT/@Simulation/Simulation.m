classdef Simulation < handle
    % SIMULATION ��ü
    %   ���ǽ����� �����ϱ� ���Ͽ� �ʿ��� �⺻���� ������� �Լ����� �Ҵ�Ǿ� ����
    
    properties
                                       %  ������                  ����                                           ����
                                       % ===============================================================================================================
        deltaT                         % �ð�����                 ����                second. �ùķ��̼� ����� �ҿ�������(discrete) �ð��� ������ �ǹ� (time step)
        iniTime                        % �ʱ�ð�                 ����                second. �ùķ��̼� ������ ������ ���� �ð� (������ 0���� ����)
        totTime                        % �����ð�                 ����       second. �ùķ��̼��� �����ϴ� �ִ����� �ð�����, �� �ð��� �Ѿ�� ������ ���ǽ����� �ߴܽ�Ų��.
        
        chkWayptRange                  % ����������������        ����       meter�ݰ�. ����������� look-ahead point�� �ƴ� ����κ��� ���� �����ؾ� �� 
        WayptIdx                       % ������ε���             ����      ����� �� ���� ���� ���� ������� ������ ��� ���� (�� ��° ������� �����ϰ� �ִ°�?)
        
        iterSim                        % ���ǽ���ݺ�ȸ��          ����        ���ǽ����� �κ��� ������ ���� or �����ð� ���� ���� ��� ����Ǵµ�
                                       %                                        �� �� ���ǽ����� deltaT�� ���߾� ������ ���� ȸ���� �ǹ�. {����ð�} = {iterSim} X {deltaT}
    end
    
    
    
    
    methods
        function setSim(sim, dt, t0, totT, range)                            % ���ǽ��� �����Ҵ� �� �ʱ�ȭ : input�� �޾Ƽ� ���ǽ��迡 �ʿ��� �ð���, �ӹ�����䱸 ������ �Ҵ�
           sim.deltaT = dt;
           sim.iniTime = t0;
           sim.totTime = totT;
           sim.chkWayptRange = range;
           sim.WayptIdx = 1;
           sim.iterSim = 1;
        end
        
        function evaluateSim(sim, env, veh, ctrl)                            % ���ǽ��� ���� : Simulation, Environment, Vehicle, Control ��ü�� ������ Ȱ���Ͽ� ���ǽ��� ����
            % �ʱ⿡�� ������ ������ �ؾ� �ϹǷ� while ���� ������ TRUE�� �д�.
            procDet=1;
            % ���ǽ��� ������ ũ�� �� ���� while loop������ �����ȴ�.
            % 1. ����������� �ٴٸ� ������ ����
            % 2. look-ahead point�� ����������� �ٴٸ� ������ ����
            while (sim.WayptIdx<env.nWaypt+1)
                while (procDet)
                    
                    % ���� �� �ӵ� ����
                    SteeringControl(ctrl,env,veh,sim);
                    SpeedControl(ctrl,env,veh,sim);

                    % ����κ��� �ӵ��� �ⱸ���� ���� ��� �� ������Ʈ
                    CalculateSpeed(veh,ctrl,sim);
                    CalculateState(veh,ctrl,sim);
                    
                    % control ��ü�� ���� ������ �ش� ��ü ���ο� �ִ� �����(history)�� ������Ʈ�ϰ�, ���ǽ��� �ݺ�ȸ���� �ø���.
                    setHistoryControlState(ctrl,sim);
                    sim.iterSim=sim.iterSim+1;
                    
                    % ���ǽ��� ����ð��� �����ð��� �ʰ��ϸ� ����.
%                     if (sim.iterSim > sim.totTime/sim.deltaT)
%                         break;
%                     end
                    
                    % �ռ� ����� while loop�� �� 2���� ������ ������Ű�� ���Ͽ� ����Ѵ�.
                    %  ���� ���� �����ϴ� ������� ����������� �ƴϸ�, ����������� look-ahead point ������ �Ÿ��� look-ahead point �Ÿ����� �������� �Ǻ��ϰ�,
                    %  �׷��� �ʴٸ� ����������� ����κ� ������ �Ÿ��� �̸� �Ҵ��� ������������������ ���ԵǾ� �ִ����� �Ǻ��Ѵ�.
                    procDet = (sim.WayptIdx ~= env.nWaypt)*(norm(env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2)) > ctrl.lfw)+...
                        (sim.WayptIdx == env.nWaypt)*(norm(env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2)) > sim.chkWayptRange);
                end
               
                % ���� ����������� �Ҵ��ϱ� ���� ������ε����� �ø���, 
                % ���� ���� ����������� �����ϱ� ���� �ʱ� ���������� ������ ������ �ؾ� �ϹǷ� while ���� ������ TRUE�� �д�.
                sim.WayptIdx=sim.WayptIdx+1;
                procDet=1;
            end
        end
    end
    
end

