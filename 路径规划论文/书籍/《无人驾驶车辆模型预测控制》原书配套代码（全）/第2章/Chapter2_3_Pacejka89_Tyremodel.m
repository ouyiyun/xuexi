%Pacejka'89��̥ģ����Ϊ��̥�ڴ�ֱ���������������Եġ�����Ϊ����
%���ڲ�����ٶȳ�����Χ��0.4g����ƫ�ǡ�5����龰�¶Գ�����̥���кܸߵ���Ͼ���
%�����ʽ��û�п��ǵ���Ħ��ϵ����Ӱ��
%%
Fz=5;%��ֱ�غɣ���λ��KN
%Longitudinal Force(pure longitudinal slip)  
%input:slip ratio s 
s=linspace(-20,20,40);                       %-20~20��ֵ40�������ɻ����ʺ�����
%*************longitudinal coefficients*******************************%
b0=2.37272;
b1=-9.46;
b2=1490;
b3=130;
b4=276;
b5=0.0886;
b6=0.00402;
b7=-0.0615;
b8=1.2;
b9=0.0299;
b10=-0.176;
%**********parameters *********************%
Cx=b0;%������״����
Dx=b1*Fz^2+b2*Fz;%����������
BCDx=(b3*Fz^2+b4*Fz)*exp(-b5*Fz);%��������㴦������ն�
Bx=BCDx/(Cx*Dx);%�ն�����
Shx=b9*Fz+b10;%���ߵ�ˮƽ����Ư��
kx=s+Shx;%�������X1
Svx=0;%%���ߵĴ�ֱ����Ư��
Ex=b6*Fz^2+b7*Fz+b8;%���ߵ���������
Fx=Dx*sin(Cx*atan(Bx*kx-Ex*(Bx*kx-atan(Bx*kx))))+Svx; %�������ļ��㹫ʽ

figure (1);
plot(s,Fx);%��������������
grid  on  %����դ��
set(gca,'xlim',[-20 20]);                              %����x�᷶Χ �̶�
set(gca,'xtick',[-20:5:20]);                          %����x���� 
set(gca,'ylim',[-8000 8000])                        %����x�᷶Χ 
set(gca,'ytick',[-8000:1000:8000]);                   %����x���� 
xlabel('��������'); 
ylabel('������/��N��'); 
title('������(���ݻ�)');

%%
%Lateral Force(pure side slip)  
%input:�����ƫ �ǲ�ƫ�� alpha
alpha=linspace(-8,8,16);   %-8~8��ֵ16�������ɲ�ƫ�Ǻ�����
r=0;  %����ǣ���Ϊ��
%*************lateral coefficients*******************************%
a0 = 1.65;
a1 = -34;
a2 = 1250;
a3 = 3036;
a4 = 12.8;
a5 = 0.00501;
a6 = -0.02103;
a7 = 0.77394;
a8 = 0.0022890;
a9 = 0.013442;
a10 = 0.003709;
a11 = 19.1656;
a12 = 1.21356;
a13 = 6.26206;

%**********parameters *********************%
Cy=a0;%������״����
Dy=a1*Fz^2+a2*Fz;%����������
BCDy=a3*sin(2*atan(Fz/a4))*(1-a5*abs(r));%��������㴦�Ĳ���ն�
By=BCDy/(Cy*Dy);%�ն�����
Shy=a9*Fz+a10+a8*r;%���ߵ�ˮƽ����Ư��
ky=alpha+Shy;%�������X
Svy=a11*Fz*r+a12*Fz+a13;%���ߵĴ�ֱ����Ư��
Ey=a6*Fz^2+a7;%������������
%********************lateral force formulation
Fy0=Dy*sin(Cy*atan(By*ky-Ey*(By*ky-atan(By*ky))))+Svy; %�������ļ��㹫ʽ

figure (2);
plot(alpha,Fy0);
grid  
set(gca,'xlim',[-8 8]);                              %����x�᷶Χ 
set(gca,'xtick',[-8:1:8]);                          %����x���� 
set(gca,'ylim',[-8000 8000])                        %����x�᷶Χ 
set(gca,'ytick',[-8000:1000:8000]);                   %����x���� 
xlabel('��ƫ��'); 
ylabel('������/��N��'); 
title('������(����ƫ)');

%%
%%Aligning Torque(pure side slip) 
%input:��ƫ�� 
%**********************ALIGNING_COEFFICIENTS*****************%
 c0 = 2.34000;
 c1 = 1.4950;
 c2 = 6.416654;
 c3 = -3.57403;
 c4 = -0.087737;
 c5 = 0.098410;
 c6 = 0.0027699;
 c7 = -0.0001151;
 c8 = 0.1000;
 c9 = -1.33329;
 c10 = 0.025501;
 c11 = -0.02357;
 c12 = 0.03027;
 c13 = -0.0647;
 c14 = 0.0211329;
 c15 = 0.89469;
 c16 = -0.099443;
 c17 = -3.336941;
%**********parameters *********************%
Cz=c0;%������״����
Dz=c1*Fz^2+c2*Fz;%����������
BCDz=(c3*Fz^2+c4*Fz)*(1-c5*abs(r))*exp(-c5*Fz);%����������㴦��Ťת�ն�
Bz=BCDz/(Cz*Dz);%�ն�����
Shz=c11*r+c12*Fz+c13;%���ߵ�ˮƽ����Ư��
kz=alpha+Shz;%�������X
Svz=r*(c14*Fz^2+c15*Fz)+c16*Fz+c17;%���ߵĴ�ֱ����Ư��
Ez=(c7*Fz^2+c8*Fz+c9)*(1-c10*abs(r));%������������
%********************aligning torque formulation
Mz0=Dz*sin(Cz*atan(Bz*kz-Ez*(Bz*kz-atan(Bz*kz))))+Svz; %�������ļ��㹫ʽ

figure (3);
plot(alpha,Mz0);
grid  
set(gca,'xlim',[-8 8]);                         %����x�᷶Χ 
set(gca,'xtick',[-8:1:8]);                      %����x���� 
set(gca,'ylim',[-80 80])                        %����x�᷶Χ 
set(gca,'ytick',[-80:10:80]);                   %����x���� 
xlabel('��ƫ��'); 
ylabel('��������/��N��'); 
title('��������(����ƫ)');