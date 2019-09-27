function [sys,x0,str,ts] = MPC_Controller(t,x,u,flag)
% �ó����ܣ���LTV MPC �ͳ����򻯶���ѧģ�ͣ�С�Ƕȼ��裩��ƿ��������������Թ滮���������켣��ʵ�ֹ켣���ٿ��ƹ���
% ����汾 V1.0��MATLAB�汾��R2011a,����S�����ı�׼��ʽ��
% �����д���� 2013.12.17
% ���һ�θ�д 2013.12.18
% ״̬��=[y_dot,x_dot,phi,phi_dot,Y,X]��������Ϊǰ��ƫ��delta_f
% QP�ⷨ������֮ǰ�ķ�����������falconeѧλ�����еķ�������֮ǰ�ķ�����ʵ�ȼۣ�
%

switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
%  case 4
%   sys = mdlGetTimeOfNextVarHit(t,x,u); % Get next sample time 

 case {1,4,9} % Unused flags
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

sizes = simsizes;
sizes.NumContStates  = 0; %
sizes.NumDiscStates  = 6; %
sizes.NumOutputs     = 1;
sizes.NumInputs      = 16;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001];    
global U;
U=[0];%��������ʼ��,���������һ�������켣����������ȥ����UΪһά��
% global x;
% x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.02 0];       % sample time: [period, offset]
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global a b; 
    %global u_piao;
    global U;
    %global kesi;
    tic
    Nx=6;%״̬���ĸ���
    Nu=1;%�������ĸ���
    Ny=2;%������ĸ��������falcone��LTV������֤�����������û������������Ŀ���Ч���ã���߿����������������
    Np =25;%Ԥ�ⲽ��
    Nc=10;%���Ʋ���
    Row=1000;%�ɳ�����
    fprintf('Update start, t=%6.3f\n',t)
   
    %����ӿ�ת��, x_dot�����һ���ǳ�С�������Ƿ�ֹ���ַ�ĸΪ������
   % y_dot=u(1)/3.6-0.000000001*0.4786; %CarSim�������km/h��ת��Ϊm/s
    y_dot=u(1)/3.6;
    x_dot=u(2)/3.6+0.0001;%CarSim�������km/h��ת��Ϊm/s
    phi=u(3)*3.141592654/180; %CarSim�����Ϊ�Ƕȣ��Ƕ�ת��Ϊ����
    phi_dot=u(4)*3.141592654/180;
    Y=u(5);%��λΪm
    X=u(6);%��λΪ��
    
    
%% ���ݹ滮��������ȷ���ο��켣
%�Ӷ˿ڶ���ֲ��滮��������,���ݽ�������4�����ߵĲ���
    Paramater_X_Y(1,1)=u(7);
    Paramater_X_Y(1,2)=u(8);
    Paramater_X_Y(1,3)=u(9);
    Paramater_X_Y(1,4)=u(10);
    Paramater_X_Y(1,5)=u(11);
    Paramater_X_phi(1,1)=u(12);
    Paramater_X_phi(1,2)=u(13);
    Paramater_X_phi(1,3)=u(14);
    Paramater_X_phi(1,4)=u(15);
    Paramater_X_phi(1,5)=u(16);

%% ������������
%syms sf sr;%�ֱ�Ϊǰ���ֵĻ�����,��Ҫ�ṩ
    Sf=0.2; Sr=0.2;
%syms lf lr;%ǰ���־��복�����ĵľ��룬�������в���
    lf=1.232;lr=1.468;
%syms C_cf C_cr C_lf C_lr;%�ֱ�Ϊǰ���ֵ��ݺ����ƫ�նȣ��������в���
    Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%mΪ����������gΪ�������ٶȣ�IΪ������Z���ת���������������в���
    m=1723;g=9.8;I=4175;
   

%% �ο��켣����
%     shape=2.4;%�������ƣ����ڲο��켣����
%     dx1=25;dx2=21.95;%û���κ�ʵ�����壬ֻ�ǲ�������
%     dy1=4.05;dy2=5.7;%û���κ�ʵ�����壬ֻ�ǲ�������
%     Xs1=27.19;Xs2=56.46;%��������
    X_predict=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵ�����λ����Ϣ�����Ǽ��������켣�Ļ���
    phi_ref=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵ������켣
    Y_ref=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵ������켣
    dphi_ref=zeros(Np,1);
%% ����ת��
    %  ���¼���kesi,��״̬�������������һ��   
    kesi=zeros(Nx+Nu,1);
    kesi(1)=y_dot;%u(1)==X(1)
    kesi(2)=x_dot;%u(2)==X(2)
    kesi(3)=phi; %u(3)==X(3)
    kesi(4)=phi_dot;
    kesi(5)=Y;
    kesi(6)=X;
    kesi(7)=U(1);
    delta_f=U(1);
    fprintf('Update start, u(1)=%4.2f\n',U(1))

    T=0.02;%���沽��
    T_all=20;%��ʱ�趨���ܵķ���ʱ�䣬��Ҫ�����Ƿ�ֹ���������켣Խ��
     
    %Ȩ�ؾ������� 
    Q_cell=cell(Np,Np);
    for i=1:1:Np;
        for j=1:1:Np;
            if i==j
                %Q_cell{i,j}=[200 0;0 100;];
                Q_cell{i,j}=[2000 0;0 10000;];
            else 
                Q_cell{i,j}=zeros(Ny,Ny);               
            end
        end 
    end 
    %R=5*10^4*eye(Nu*Nc);
    R=5*10^5*eye(Nu*Nc);
    %�����Ҳ����Ҫ�ľ����ǿ������Ļ��������ö���ѧģ�ͣ��þ����복������������أ�ͨ���Զ���ѧ��������ſ˱Ⱦ���õ�
    a=[                 1 - (259200*T)/(1723*x_dot),                                                         -T*(phi_dot + (2*((460218*phi_dot)/5 - 62700*y_dot))/(1723*x_dot^2) - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2)),                                    0,                     -T*(x_dot - 96228/(8615*x_dot)), 0, 0
        T*(phi_dot - (133800*delta_f)/(1723*x_dot)),                                                                                                                  (133800*T*delta_f*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2) + 1,                                    0,           T*(y_dot - (824208*delta_f)/(8615*x_dot)), 0, 0
                                                  0,                                                                                                                                                                                  0,                                    1,                                                   T, 0, 0
            (33063689036759*T)/(7172595384320*x_dot), T*(((2321344006605451863*phi_dot)/8589934592000 - (6325188028897689*y_dot)/34359738368)/(4175*x_dot^2) + (5663914248162509*((154*phi_dot)/125 + y_dot))/(143451907686400*x_dot^2)),                                   0, 1 - (813165919007900927*T)/(7172595384320000*x_dot), 0, 0
                                          T*cos(phi),                                                                                                                                                                         T*sin(phi),  T*(x_dot*cos(phi) - y_dot*sin(phi)),                                                   0, 1, 0
                                         -T*sin(phi),                                                                                                                                                                         T*cos(phi), -T*(y_dot*cos(phi) + x_dot*sin(phi)),                                                   0, 0, 1];
   
    b=[                                                               133800*T/1723
       T*((267600*delta_f)/1723 - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot))
                                                                                 0
                                                5663914248162509*T/143451907686400
                                                                                 0
                                                                                 0];  
    d_k=zeros(Nx,1);%����ƫ���falcone42ҳ�е�d(k,t)
    state_k1=zeros(Nx,1);%Ԥ�����һʱ��״̬�������ڼ���ƫ���falcone42ҳ��kesi(k+1,t)
    %���¼�Ϊ������ɢ������ģ��Ԥ����һʱ��״̬��
    %ע�⣬Ϊ����ǰ�����ı��ʽ��a,b�����������a,b�����ͻ����ǰ�����ı��ʽ��Ϊlf��lr
    state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m);
    state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
    state_k1(3,1)=phi+T*phi_dot;
    state_k1(4,1)=phi_dot+T*((2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I);
    state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi));
    d_k=state_k1-a*kesi(1:6,1)-b*kesi(7,1);%����falcone��ʽ��2.11b�����d(k,t)
    d_piao_k=zeros(Nx+Nu,1);%d_k��������ʽ���ο�falcone(B,4c)
    d_piao_k(1:6,1)=d_k;
    d_piao_k(7,1)=0;
    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    %A=zeros(Nu+Nx,Nu+Nx);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
   % C=[0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;];%���Ǻ���������ܹ�����
    C=[0 0 1 0 0 0 0;0 0 0 0 1 0 0;];
    PSI_cell=cell(Np,1);%ԭ���汾��PHI
    THETA_cell=cell(Np,Nc);
    GAMMA_cell=cell(Np,Np);%�����µļ��㷽����ӵ�,��falcone179ҳ���һ�й�ʽ
    PHI_cell=cell(Np,1);%ע�⣬����汾��֮ǰ���е�PHI��һ����ԭ���汾��PHIΪ����汾��PSI
    for p=1:1:Np;
        PHI_cell{p,1}=d_piao_k;%��������˵�������Ҫʵʱ���µģ�����Ϊ�˼�㣬������һ�ν���
        for q=1:1:Np;
            if q<=p;
                GAMMA_cell{p,q}=C*A^(p-q);
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu);
            end 
        end
    end
    for j=1:1:Np
     PSI_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;
            else 
                THETA_cell{j,k}=zeros(Ny,Nu);
            end
        end
    end
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]
    GAMMA=cell2mat(GAMMA_cell);%��д��GAMMA
    PHI=cell2mat(PHI_cell);
    Q=cell2mat(Q_cell);
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);
    error_1=zeros(Ny*Np,1);
    Yita_ref_cell=cell(Np,1);
    for p=1:1:Np
        if t+p*T>T_all
            X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%��������ϵ�������ٶ�
            X_predict(Np,1)=X+X_DOT*Np*T;
            Y_ref(p,1)=polyval(Paramater_X_Y, X_predict(Np,1));
            phi_ref(p,1)=polyval(Paramater_X_phi, X_predict(Np,1));
            %dphi_ref(p,1)=-((2916*X_DOT*sinh((12*X_predict(Np,1))/125 - 11907/3125))/(78125*cosh((12* X_predict(Np,1))/125 - 11907/3125)^3) - (65664*X_DOT*sinh((48* X_predict(Np,1))/439 - 80922/10975))/(963605*cosh((48* X_predict(Np,1))/439 - 80922/10975)^3))/((243/(1250*cosh((12*X_predict(Np,1))/125 - 11907/3125)^2) - 684/(2195*cosh((48*X_predict(Np,1))/439 - 80922/10975)^2))^2 + 1);
            %Yita_ref_cell{p,1}=[phi_ref(p,1);dphi_ref(p,1);Y_ref(p,1)];%���Խ���ˣ���Ԥ��ʱ�������һ����ʼ����Ϊ����ֵ
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
            
        else
            X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%��������ϵ�������ٶ�
            X_predict(p,1)=X+X_DOT*p*T;%���ȼ����δ��X��λ�ã�X(t)=X+X_dot*t
            Y_ref(p,1)=polyval(Paramater_X_Y, X_predict(p,1));
            phi_ref(p,1)=polyval(Paramater_X_phi, X_predict(p,1));
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
            %dphi_ref(p,1)=-((2916*X_DOT*sinh((12* X_predict(p,1))/125 - 11907/3125))/(78125*cosh((12* X_predict(p,1))/125 - 11907/3125)^3) - (65664*X_DOT*sinh((48* X_predict(p,1))/439 - 80922/10975))/(963605*cosh((48* X_predict(p,1))/439 - 80922/10975)^3))/((243/(1250*cosh((12*X_predict(p,1))/125 - 11907/3125)^2) - 684/(2195*cosh((48*X_predict(p,1))/439 - 80922/10975)^2))^2 + 1);
            %Yita_ref_cell{p,1}=[phi_ref(p,1);dphi_ref(p,1);Y_ref(p,1)];%��3����������������һ��
        end
    end
    Yita_ref=cell2mat(Yita_ref_cell);
%     fprintf('X_predict=%4.2f\n',X_predict(1));    
%     fprintf('ʵ��λ�ã�X=%4.2f\n',X);
%     fprintf('Update Yita, phi_ref=%4.2f\n',Yita_ref(1));
%     fprintf('Update Yita, dphi_ref=%4.2f\n',Yita_ref(2));
%     fprintf('Update Yita, phi=%4.2f\n',phi);
%     fprintf('ʵ��λ�ã�Y=%4.2f\n',Y);

    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; %��ƫ��
    f_cell=cell(1,2);
    f_cell{1,1}=2*error_1'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=-cell2mat(f_cell);
    
 %% ����ΪԼ����������
 %������Լ��
    A_t=zeros(Nc,Nc);%��falcone���� P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%��Ӧ��falcone����Լ������ľ���A,������ڿ˻�
    Ut=kron(ones(Nc,1),U(1));%�˴��о�������Ŀ����ڿƻ�������,��ʱ����˳��
    umin=-0.1744;%ά������Ʊ����ĸ�����ͬ
    umax=0.1744;
    delta_umin=-0.0148*0.4;
    delta_umax=0.0148*0.4;
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    
    %�����Լ����ӲԼ�����ã���ʱ�����������ΪӲԼ��
    ycmax=[0.21;5];
    ycmin=[-0.3;-3];
    Ycmax=kron(ones(Np,1),ycmax);
    Ycmin=kron(ones(Np,1),ycmin);
    
    %��Ͽ�����Լ���������Լ��
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);THETA zeros(Ny*Np,1);-THETA zeros(Ny*Np,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};
    A_cons=cell2mat(A_cons_cell);%����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
    b_cons=cell2mat(b_cons_cell);%����ⷽ�̣�״̬������ʽԼ����ȡֵ
    
    %״̬��Լ��
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
    ub=[delta_Umax;M];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����
    
    %% ��ʼ������
%    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
       options = optimset('Algorithm','active-set');
       x_start=zeros(Nc+1,1);%����һ����ʼ��
      [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);
      fprintf('exitflag=%d\n',exitflag);
      fprintf('H=%4.2f\n',H(1,1));
      fprintf('f=%4.2f\n',f(1,1));
    %% �������
    u_piao=X(1);%�õ���������
    U(1)=kesi(7,1)+u_piao;%��ǰʱ�̵Ŀ�����Ϊ��һ��ʱ�̿���+��������
   %U(2)=Yita_ref(2);%���dphi_ref
    sys= U;
    toc
% End of mdlOutputs.


