function LQR_1()
%�����ȴӼ򵥿�ʼ������һ��ֱ�߳����ͳ���λ��ƫ�
%�ο��켣�����ɷ��������֣�
%1.������Path��ͶӰ��Ȼ����PATH��ѡȡһϵ�еĵ����ο���
%*����������������Q R�Ĳ�����ô���á�����ͨ������ô�죿*%

clear all;
close all;
clc;
%% ����������

vel = 6; % ������,��λ��m/s
L=2.85;%���
T=0.05;% sample time�� control period
% ����Բ�βο��켣
 CEN=[0,0];       % Բ��
 Radius=20;       % �뾶

%% ���ò���
Hp =10;%predictive horizion, control horizon 
N_l=200;% ���õ�������

Nx=3;%״̬���������ĸ���
Nu=1;%���Ʊ��������ĸ���

FWA=zeros(N_l,1);%ǰ��ƫ��
FWA(1,1)= 0; %��ʼ״̬��ǰ��ƫ��

x_real=zeros(Nx,N_l);%ʵ��״̬
x_real(:,1)= [22 0 pi/2]; %x0=������ʼ״̬X_init��ʼ״̬
% x_piao=zeros(N_l,Nx);%ʵ��״̬��ο��켣�����
% 
% u_real=zeros(N_l,Nu);%ʵ�ʵĿ�����
% u_piao=zeros(N_l,Nu);%ʵ�ʿ�������ο������������

% X_PIAO=zeros(N_l,3*Hp);%ͨ��DR���Ƶ�״̬
% 
% XXX=zeros(N_l,3*Hp);%���ڱ���ÿ��ʱ��Ԥ�������״ֵ̬

RefTraj=zeros(3,1);
Delta_x = zeros(3,1);

Q=[10 0 0; 0 10 0; 0 0 100];
R=[10];%r�ǶԿ���������weighting matrice

Pk=[1 0 0; 0 1 0; 0 0 1]; %��Ϊ����,�൱��QN
Vk=[0 0 0]'; %��Ϊ����,�൱��QN

%%  �㷨ʵ��
 u_feedBackward=0;
 u_feedForward=0;
 
 %*�������ɲο��켣������ͼ�����ο�*%
 [RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(x_real(1,1),x_real(1,2),CEN(1),CEN(2),Radius,250,vel,T,L);

figure(1) %���Ʋο�·��
plot(RefTraj_x,RefTraj_y,'k')
xlabel('x','fontsize',14)
ylabel('y','fontsize',14)
title('Plot of x vs y - Ref. Trajectory');
legend('reference traj');
axis equal 
grid on
hold on


for i=1:1:N_l

    G_Test = 3;
    %��ȷ���ο����ȷ������A,B.���������ΪA��B�ǲ����
    [RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(x_real(1,i),x_real(2,i),CEN(1),CEN(2),Radius,Hp,vel,T,L);
    u_feedForward = RefTraj_delta(G_Test);%ǰ��������
%     u_feedForward=0;
    Delta_x(1,1) = x_real(1,i) - RefTraj_x(G_Test);
    Delta_x(2,1) = x_real(2,i) - RefTraj_y(G_Test);
    Delta_x(3,1) = x_real(3,i) - RefTraj_theta(G_Test);
    if  Delta_x(3,1) > pi
         Delta_x(3,1) = Delta_x(3,1)-2*pi;
    else if Delta_x(3,1) < -1*pi
            Delta_x(3,1) = Delta_x(3,1) +2*pi;
        else
            Delta_x(3,1) = Delta_x(3,1);
        end            
    end
    
     % ͨ��Backward recursion ��K    
    for  j=Hp:-1:2   
        Pk_1 = Pk;
        Vk_1 = Vk;     
        A=[1    0   -vel*sin(RefTraj_theta(j-1))*T; 0    1   vel*cos(RefTraj_theta(j-1))*T; 0    0   1;];
%         B=[cos(RefTraj_theta(j-1))*T   0; sin(RefTraj_theta(j-1))*T   0; 0            vel*T/L;]; 
        COS2 = cos(RefTraj_delta(j-1))^2;
        B=[ 0 0  vel*T/(L*COS2)]'; 

        K = (B'*Pk_1*A)/(B'*Pk_1*B+R);
        Ku = R/(B'*Pk_1*B+R);
        Kv = B'/(B'*Pk_1*B+R);

        Pk=A'*Pk_1*(A-B*K)+Q;   
        Vk=(A-B*K)'*Vk_1 - K'*R*RefTraj_delta(j-1); 
    end
    
     u_feedBackward = -K*(Delta_x)-Ku*u_feedForward-Kv*Vk_1;  
    
    FWA(i+1,1)=u_feedForward+u_feedBackward;
    
     [x_real(1,i+1),x_real(2,i+1),x_real(3,i+1)]=Func_VehicleKineticModule_Euler(x_real(1,i),x_real(2,i),x_real(3,i),vel,FWA(i,1),FWA(i+1,1),T,L);  
     
    
end

%%   ��ͼ
%        figure(1);
%     plot(RefTraj_x,RefTraj_y,'b')
%     hold on;
    plot(x_real(1,:),x_real(2,:),'r*');
    title('���ٽ���Ա�');
    xlabel('����λ��X');
    % axis([-1 5 -1 3]);
    ylabel('����λ��Y');  


end