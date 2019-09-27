function [X,Y,H]=Func_VehicleKineticModule_Euler(x,y,heading,vel,FWA,DFWA,T,L)
%�����˶�ѧģ�ͣ�״̬����x,y,heading;��������vel=constant��FWA
%�̶��Ĳ������������ֵ��

%%
%initial the status of the vehicle
num=100;
Xmc=zeros(1,num);
Ymc=zeros(1,num);
Headingmc=zeros(1,num);
Xmc(1)=x;
Ymc(1)=y;%x,y��ʼ����
Headingmc(1)=heading;%����,

Headingrate=zeros(1,num);
FrontWheelAngle=zeros(1,num);

t=T/num;
%%
FrontWheelAngle=linspace(FWA,DFWA,num);%ǰ��ƫ��
Headingrate=vel*tan(FrontWheelAngle)/L;
for i=2:num
    Headingmc(i)=Headingmc(i-1)+Headingrate(i)*t;
    Xmc(i)=Xmc(i-1)+vel*t*cos(Headingmc(i-1));
    Ymc(i)=Ymc(i-1)+vel*t*sin(Headingmc(i-1));
end
%%
    X=Xmc(num);
    Y=Ymc(num);
    H=Headingmc(num);
end

%% test
% [X,Y,H]=VehicleKineticModule_Euler(0,0,0,10,0,3,0.1,2.85)
%plot(X,Y,'b');

