function [RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(Pos_x,Pos_y,CEN_x,CEN_y,Radius,N,Velo,Ts,L)
%RefTrajΪҪ���ɵĲο�·��
%Pos_x,Pos_yΪ��������
%CEN_x,CEN_y,RadiusԲ����뾶
%NҪ���ɼ����ο��㣬��Ԥ��ռ䡣
%Velo,Ts���������ʱ��
%L���������
RefTraj=zeros(N,4);%���ɵĲο�·��
Alpha_init=Func_Alpha_Pos(CEN_x,CEN_y,Pos_x,Pos_y);%���ȸ��ݳ���λ�ú�Բ��ȷ��alpha

Omega=Velo/Radius%��֪���ٺͰ뾶��������ý��ٶȡ�

DFWA=atan(L/Radius);

for k=1:1:N
    Alpha(k)=Alpha_init+Omega*Ts*(k-1);
    RefTraj(k,1)=Radius*cos(Alpha(k))+CEN_x;%x
    RefTraj(k,2)=Radius*sin(Alpha(k))+CEN_y;%y
    RefTraj(k,3)=Func_Theta_Pos(Alpha(k));%theta  
 
    RefTraj(k,4)=DFWA;%ǰ��ƫ�ǣ����Ե���ǰ����

end
RefTraj_x= RefTraj(:,1);
RefTraj_y= RefTraj(:,2);
RefTraj_theta= RefTraj(:,3);
RefTraj_delta= RefTraj(:,4);

end