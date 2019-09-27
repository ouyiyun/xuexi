close all;
clear all;
clc;
% ���ױ���������
% ����4.5m ��1.8 

% ��Сת��뾶8��Ӧ�������
veh_k_max=1/8;
% ��ʼλ��
XS=[0 0 90];
% Ŀ��λ��
XG=[40 30 270];

% ������Χ
d_max=50;

% �����ߵ�����
all_k=[];

n=abs(XS(1)-XG(1))*1000+1;
ds=[];
best_ds=inf;
best_d1=0;
best_d2=0;

for d1=0:0.5:d_max
    for d2=0:0.5:d_max
        % ����p1��p2
        p1=point_position(d1,XS);
        p2=point_position(-1*d2,XG);

        t=linspace(0,1,n);
        x=XS(1).*(1-t).^3+3*p1(1).*t.*(1-t).^2+p2(1).*t.^2.*(1-t)+XG(1).*t.^3;
        y=XS(2).*(1-t).^3+3*p1(2).*t.*(1-t).^2+p2(2).*t.^2.*(1-t)+XG(2).*t.^3;
    %     plot(x,y,'g-');

        % ��������
        x_diff1=3*(p1(1)-XS(1)).*(1-t).^2+6*(p2(1)-p1(1)).*(1-t).*t+3*(XG(1)-p2(1)).*t.^2;
        y_diff1=3*(p1(2)-XS(2)).*(1-t).^2+6*(p2(2)-p1(2)).*(1-t).*t+3*(XG(2)-p2(2)).*t.^2;
        x_diff2=6*(p1(1)-XS(1)).*(1-t)+6*(p2(1)-p1(1)).*(1-2.*t)+6*(XG(1)-p2(1)).*t;
        y_diff2=6*(p1(1)-XS(1)).*(1-t)+6*(p2(1)-p1(1)).*(1-2.*t)+6*(XG(1)-p2(1)).*t;
        k=abs(x_diff1.*y_diff2-x_diff2.*y_diff1)./(x_diff1.^2+y_diff1.^2).^1.5;

        % ���㻡��
        f=(x_diff1.^2+y_diff1.^2).^0.5;
        s=sum(f./(n-1));
        ds=[ds s];

        k_max=max(k);
        k_min=min(k);
        k_diff=k_max-k_min;

        %  ͬʱ��ע���ʺ�·��
        if k_max<=veh_k_max
            if k_diff<(0.64*veh_k_max)
                if best_ds>s
                    best_ds=s;
                    best_d1=d1;
                    best_d2=d2;
                end
            end
        end
    end
end

p1=point_position(best_d1,XS);
p2=point_position(-1*best_d2,XG);

x_=[XS(1) p1(1) p2(1) XG(1)];
y_=[XS(2) p1(2) p2(2) XG(2)];
hold on
plot(x_,y_,'r*:');

t=linspace(0,1,n);
x_=XS(1).*(1-t).^3+3*p1(1).*t.*(1-t).^2+p2(1).*t.^2.*(1-t)+XG(1).*t.^3;
y_=XS(2).*(1-t).^3+3*p1(2).*t.*(1-t).^2+p2(2).*t.^2.*(1-t)+XG(2).*t.^3;
x_diff1=3*(p1(1)-XS(1)).*(1-t).^2+6*(p2(1)-p1(1)).*(1-t).*t+3*(XG(1)-p2(1)).*t.^2;
y_diff1=3*(p1(2)-XS(2)).*(1-t).^2+6*(p2(2)-p1(2)).*(1-t).*t+3*(XG(2)-p2(2)).*t.^2;
yaw=rad2deg(asin(y_diff1./(x_diff1.^2+y_diff1.^2).^0.5));
plot(x_,y_,'r-');

% ������λ��
% d ����ľ���
% X λ��
function p=point_position(d, X)
theta=deg2rad(X(3)-90);
x_=0;
y_=d;
p(1)=x_*cos(theta)-y_*sin(theta)+X(1);
p(2)=x_*sin(theta)+y_*cos(theta)+X(2);
end