function [sys,x0,str,ts] = MPC_TrajPlanner(t,x,u,flag)
% �ó����ܣ��õ�����ģ����ƹ滮�ڣ��ܹ�����ϰ���
% ����汾 V1.0��MATLAB�汾��R2011a,����S�����ı�׼��ʽ��
% �����д���� 2013.12.17
% ���һ�θ�д 2014.02.24
% ״̬��=[y_dot,x_dot,phi,,Y,X]��������Ϊǰ��ƫ��ay


switch flag,
 case 0  %flag=0��ʾ���ڳ�ʼ��״̬����ʱ�ú���mdlInitializeSizes���г�ʼ��
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2  %flag=2��ʾ��ʱҪ������һ����ɢ״̬
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3 %flag=3��ʾ��ʱҪ�������
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
%  case 4
%   sys = mdlGetTimeOfNextVarHit(t,x,u); % Get next sample time 

 case {1,4,9} % Unused flags
    % flag=1��ʾ��ʱҪ��������״̬��΢��
    %flag=4��ʾ��ʱҪ������һ�β�����ʱ�䣬ֻ����ɢ����ϵͳ�����ã���Ҫ���ڱ䲽��������
    %flag=9��ʾ��ʱϵͳҪ������һ����˵д����mdlTerminate������д��sys=[]�Ϳ�
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes; %��������ģ������Ľṹ����simsizes������
sizes.NumContStates  = 0;%ģ������״̬�����ĸ���
sizes.NumDiscStates  = 5;%ģ����ɢ״̬�����ĸ���
sizes.NumOutputs     = 10;%���������Y��phi,��������������ߵ�ϵ���������Ҫ���Կ���dphi,dphi=ay/x_dot;
sizes.NumInputs      = 6;%ģ����������ĸ���
sizes.DirFeedthrough = 1; %ģ���Ƿ����ֱ�ӹ�ͨ
sizes.NumSampleTimes = 1;%ģ��Ĳ���������������һ��
sys = simsizes(sizes); %������󸳸�sys���

x0 =[0.001;0.0001;0.0001;0.00001;0.00001;];    %״̬��������
%global U;
%U=[0];%��������ʼ��,UΪһά��
% global x;
% x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   
% Initialize the discrete states.
str = [];             %���������� Set str to an empty matrix.
ts  = [0.1 0];       % ��������: [period, offset],����켣�滮��������Ϊ100ms
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x; %����״̬����
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    %u��CarSim�����
    %x��״̬����x=[y_dot,x_dot,phi,,Y,X]
    %t��ʱ�����
    tic
    Nx=5;%״̬���ĸ����� 
    %Nu=1;%�������ĸ�����������Ϊǰ��ƫ��ay
    %Ny=2;%������ĸ��������falcone��LTV������֤�����������û������������Ŀ���Ч���ã���߿����������������
    Np =15;%Ԥ�ⲽ��
    Nc=2;%���Ʋ���
    Nobs=6;%�ϰ������
    T=0.1;%Sample Time
   
    %����ӿ�ת��,x_dot�����һ���ǳ�С�������Ƿ�ֹ���ַ�ĸΪ������
    % y_dot=u(1)/3.6-0.000000001*0.4786;%CarSim�������km/h��ת��Ϊm/s
    y_dot=u(1)/3.6; %�ٶȵ�λ��km/h��ת��Ϊm/s
    x_dot=u(2)/3.6+0.0001;%CarSim�������km/h��ת��Ϊm/s.��һ����С������Ϊ�˷�ֹ��ĸΪ��
    phi=u(3)*pi/180; %CarSim�����Ϊ�Ƕȣ��Ƕ�ת��Ϊ����
    phi_dot=u(4)*pi/180;%���ٶȣ��Ƕ�ת��Ϊ����
    Y=u(5);%��λΪm
    X=u(6);%��λΪ��
   
%% �ο��켣����
    shape=2.4;%�������ƣ����ڲο��켣����
    dx1=25;dx2=21.95;%û���κ�ʵ�����壬ֻ�ǲ�������
    dy1=4.05;dy2=5.7;%û���κ�ʵ�����壬ֻ�ǲ�������
    Xs1=27.19;Xs2=56.46;%��������
  	X_phi=1:1:220;%�����������Ǹ��������ٶȣ�x_dot�������ģ�����ٶ�Ϊ10m/s������=10*0.1=1
    z1=shape/dx1*(X_phi-Xs1)-shape/2;
    z2=shape/dx2*(X_phi-Xs2)-shape/2;
    Y_ref=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));
   % phi_ref=atan(dy1*(1./cosh(z1)).^2*(1.2/dx1)-dy2*(1./cosh(z2)).^2*(1.2/dx2));
%% ����ת������״̬����ת��Ϊ״̬��������
    State_Initial=zeros(Nx,1);
    State_Initial(1,1)=y_dot;
    State_Initial(2,1)=x_dot;
    State_Initial(3,1)=phi;
    State_Initial(4,1)=Y;
    State_Initial(5,1)=X;    
%% �ϰ�����Ϣ����
    X_obstacle=zeros(Nobs,1);
    X_obstacle(1:2)=30;
    X_obstacle(3:4)=35;
    X_obstacle(5:6)=32.5;
    Y_obstacle=zeros(Nobs,1);
    Y_obstacle(1)=0.5;
    Y_obstacle(2)=1;
    Y_obstacle(3)=0.5;
    Y_obstacle(4)=1;
    Y_obstacle(5)=0.5;
    Y_obstacle(6)=1;
    Yref=(Y_ref(1,round(State_Initial(5,1))+1:round(State_Initial(5,1))+15))';%Yref���õ��ǽ����㷨���˴�Ϊ�ֲ�����·��
    Q=100*eye(Np,Np);%�����������۾��󣬶���Ϊ��1�����Ը��ݸ���������Ե���
    R=20*eye(Nc,Nc); %
    S=100;%���Ϻ�����Ȩ��    
%% ��ʼ������
    %����Լ��
    mu=0.4;%����Ħ��ϵ��
    g=9.8;
    lb=[-mu*g;-mu*g];
    ub=[mu*g;mu*g];
    A=[];
    b=[];
    Aeq=[];
    beq=[];
    options = optimset('Algorithm','active-set');
    [A,fval,exitflag]=fmincon(@(x)MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle),[0;0;],A,b,Aeq,beq,lb,ub,[],options);%��Լ����⣬���ٶ���
%   [A,fval,exitflag]=fminbnd(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Yref,Q,R,S),lb,ub);%ֻ�����½�Լ��������������ֲ���С
%   [A,fval,exitflag]=fminsearch(@(x)MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle),[0;0]);%��Լ����⣬�ٶ����
    fprintf('exitflag=%d\n',exitflag);
   
%% �������
% ���¸��ݼ�����Ŀ������Ƶ����е�״̬��
    y_dot_predict=zeros(Np,1);
    x_dot_predict=zeros(Np,1);
    phi_predict=zeros(Np,1);
    Y_predict=zeros(Np,1);
    X_predict=zeros(Np,1);

 for i=1:1:Np
     if i==Nc-1 
            ay(i)=A(1);
             % �������״̬������
            y_dot_predict(i,1)=State_Initial(1,1)+T*ay(i);
            x_dot_predict(i,1)=State_Initial(2,1);
            phi_predict(i,1)=State_Initial(3,1)+T*ay(i)/State_Initial(2,1);
            Y_predict(i,1)=State_Initial(4,1)+T*(State_Initial(2,1)*sin(State_Initial(3,1))+State_Initial(1,1)*cos(State_Initial(3,1)));
            X_predict(i,1)=State_Initial(5,1)+T*(State_Initial(2,1)*cos(State_Initial(3,1))-State_Initial(1,1)*sin(State_Initial(3,1)));  
      else %if i<=5
            ay(i)=A(2);%����д���ǽ�������������������
            y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i);
            x_dot_predict(i,1)=State_Initial(2,1);
            phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
            Y_predict(i,1)=Y_predict(i-1)+T*(State_Initial(2,1)*sin(phi_predict(i-1))+y_dot_predict(i-1)*cos(phi_predict(i-1)));
            X_predict(i,1)=X_predict(i-1)+T*(State_Initial(2,1)*cos(phi_predict(i-1))-y_dot_predict(i-1)*sin(phi_predict(i-1)));
      end 
 end
    Paramater_X_Y=polyfit(X_predict,Y_predict,4);
    Paramater_X_PHI=polyfit(X_predict,phi_predict,4);
    OutPut(1:5)=Paramater_X_Y;
    OutPut(6:10)=Paramater_X_PHI;
    sys=OutPut;
    toc
% End of mdlOutputs.
 %% ����ۺ����Ĺ����Ӻ���
function cost = MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle)
    cost=0;
    y_dot=State_Initial(1,1);
    x_dot=State_Initial(2,1);
    phi=State_Initial(3,1);
    Y=State_Initial(4,1);
    X_start=State_Initial(5,1);
    
    y_dot_predict=zeros(Np,1);
    x_dot_predict=zeros(Np,1);
    phi_predict=zeros(Np,1);
    Y_predict=zeros(Np,1);
    X_predict=zeros(Np,1);
    Y_error=zeros(Np,1);
    J_obst=zeros(Np,1);
    ay=zeros(Np,1);
    
    for i=1:1:Np
        if i==Nc-1 
            ay(i,1)=x(1);
             % �������״̬������
            y_dot_predict(i,1)=y_dot+T*ay(i,1);
            x_dot_predict(i,1)=x_dot;
            phi_predict(i,1)=phi+T*ay(i)/x_dot;
            Y_predict(i,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
            X_predict(i,1)=X_start+T*(x_dot*cos(phi)-y_dot*sin(phi));
            for j=1:1:Nobs
                J_obst(i,1)=J_obst(i,1)+1/(((X_predict(i,1))-X_obstacle(j,1))^2+(Y_predict(i,1)-Y_obstacle(j,1))^2+0.000001);
            end
        else %if i<=5
            ay(i,1)=x(2);%����д���ǽ�������������������
            y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i,1);
            x_dot_predict(i,1)=x_dot;
            phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
            Y_predict(i,1)=Y_predict(i-1)+T*(x_dot*sin(phi_predict(i-1))+y_dot_predict(i-1)*cos(phi_predict(i-1)));
            X_predict(i,1)=X_predict(i-1)+T*(x_dot*cos(phi_predict(i-1))-y_dot_predict(i-1)*sin(phi_predict(i-1)));
            for p=1:1:Nobs
                J_obst(i,1)=J_obst(i,1)+1/(((X_predict(i,1))-X_obstacle(p,1))^2+(Y_predict(i,1)-Y_obstacle(p,1))^2+0.000001);
            end
%             else 
%             ay(i)=x(2);
%             y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i);
%             x_dot_predict(i,1)=x_dot;
%             phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
%             Y_predict(i,1)=Y_predict(i-1)+T*(x_dot*sin(phi_predict(5,1))+y_dot_predict(5,1)*cos(phi_predict(5,1)));
%             X_predict(i,1)=X_predict(i-1)+T*(x_dot*cos(phi_predict(5,1))-y_dot_predict(5,1)*sin(phi_predict(5,1)));
%             end 
        end
    %J_obst=J_obst+1/(((X_predict(i,1))-X_obstacle(2,1))^2+(Y_predict(i,1)-Y_obstacle(2,1))^2+0.00001);
        Y_error(i,1)=Y_predict(i,1)-Yref(i,1);%ע��˴�Yref��Y_refҪ���ֿ�����Yref�Ǿֲ�����·��,Y_refΪȫ������·��
    end 
        cost=cost+Y_error'*Q*Y_error+ay(1:2)'*R*ay(1:2)+S*sum(J_obst(:));
% End of CostFunction
