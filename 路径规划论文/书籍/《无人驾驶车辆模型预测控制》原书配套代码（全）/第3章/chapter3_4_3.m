
clc;
clear all;
%% �ο��켣����
tic
Nx=3;%״̬������
Np=25;%Ԥ��ʱ��
Nc=2;%����ʱ��
l=1;
N=100;%�ο��켣������
T=0.05;%��������
Xref=zeros(Np,1);
Yref=zeros(Np,1);
PHIref=zeros(Np,1);

%% �����ܺ����������ĳ�ʼ��
State_Initial=zeros(Nx,1);%state=[y_dot,x_dot,phi,Y,X],�˴�Ϊ������ʼֵ
State_Initial(1,1)=0;%x 
State_Initial(2,1)=0;%y
State_Initial(3,1)=pi/6;%phi

Q=100*eye(Np+1,Np+1);
R=100*eye(Np+1,Np+1);

%% ��ʼ���
for j=1:1:N
    lb=[0.8;-0.44;0.8;-0.44];
    ub=[1.2;0.44;1.2;0.44];
    A=[];
    b=[];
    Aeq=[];
    beq=[];
    for Nref=1:1:Np
        Xref(Nref,1)=(j+Nref-1)*T;
        Yref(Nref,1)=2;
        PHIref(Nref,1)=0;
    end
    options = optimset('Algorithm','active-set');
    [A,fval,exitflag]=fmincon(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Xref,Yref,PHIref,Q,R),[0;0;0;0;],A,b,Aeq,beq,lb,ub,[],options);%��Լ����⣬���ٶ���
    %[A,fval,exitflag]=fminbnd(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Yref,Q,R,S),lb,ub);%ֻ�����½�Լ��������������ֲ���С
    %[A,fval,exitflag]=fminsearch(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Xref,Yref,PHIref,Q,R),[0;0;0;0]);%��Լ����⣬�ٶ����
    v_actual=A(1);
    deltaf_actual=A(2);
%     fval
%     exitflag
    X00(1)=State_Initial(1,1);
    X00(2)=State_Initial(2,1);
    X00(3)=State_Initial(3,1);
    XOUT=dsolve('Dx-v_actual*cos(z)=0','Dy-v_actual*sin(z)=0','Dz-v_actual*tan(deltaf_actual)=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
    t=T;
    State_Initial(1,1)=eval(XOUT.x);
    State_Initial(2,1)=eval(XOUT.y);
    State_Initial(3,1)=eval(XOUT.z);
    
    figure(1)
    plot(State_Initial(1,1),State_Initial(2,1),'b*');
    axis([0 5 0 3]);
    hold on;
    plot([0,5],[2,2],'r--');
    hold on;
    
end 
 toc
