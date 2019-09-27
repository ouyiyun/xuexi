%function astar(map,dataNode)
tic
%=============����=====================
 load ('map.mat');
 map =map([1:10],[1:10]);
%  block = [  
%     0,1,1,0,0,0,0,1,1,0;  
%     0,1,1,0,0,0,1,0,0,0;  
%     0,0,0,0,0,0,1,1,1,0; 
%     0,0,0,0,0,0,0,0,0,0;  
%     0,0,0,0,0,0,0,0,0,0;  
%     0,0,0,0,0,0,0,0,0,0;  
%     0,0,0,0,0,0,0,0,0,0;  
%     0,0,0,0,0,0,0,0,0,0;  
%     0,0,0,0,0,0,0,0,0,0;  
%     0,0,0,0,0,0,0,0,0,0;  
% ];
% map = block;
map=fliplr(map')

startNode=[5 5];
endNode =[7 7];
%endNode =[9 10];
dataNode=[startNode;endNode]
%================================
m = 10;n = 10;%��ʾ��������ǽ�ĵ�ͼ ����  ���� 
 Spoint = [2 2];% ��ʼ��
%Spoint = [3 3];
Epoint = [m+1 n+1];%���ܵ�
 %%//�ϰ���   
 block = [  
    0,1,0,0,0,0,0,0,0,0;  
    0,1,1,0,1,1,1,0,0,0;  
    0,0,0,0,0,0,0,0,0,0; 
    1,1,1,0,1,0,0,0,0,0;  
    0,1,0,0,1,0,1,1,1,0;  
    0,1,0,0,1,1,1,1,1,0;  
    0,0,0,1,1,0,0,0,1,0;  
    0,1,0,0,0,0,1,0,1,0;  
    0,1,1,1,0,1,1,0,1,1;  
    0,0,0,0,0,0,1,0,0,0;  
]; 
%========map �п��Ƿ���Ч========
  [m n] = size(map);
 if ([m n] ~= [0 0])
  block =map;
  m = m;
  n = n;%��ʾ��������ǽ�ĵ�ͼ ����  ���� 
 else
     if([dataNode(2,1) dataNode(2,2)] > [m n])
        dataNode(2,:)= [m n]
     end
 end
 Spoint = dataNode(1,:)+1;% ��ʼ��
 Epoint =dataNode(2,:)+1;%���ܵ�
%%��ͼ
for i = 1:m+2% ��ʾ�������ҵ�ǽ
    if i == 1
        for j = 1:n+2
            Matrix(i,j) = -inf;%ǽ
        end
    elseif i == m+2
        for j = 1:n+2%ǽ
            Matrix(i,j) = -inf;
        end
    else
        for j = 1:n+2
            if ((j == 1)|(j == n+2))
                Matrix(i,j) = -inf;%ǽ
            else
                Matrix(i,j) = inf;%���������� ��ʾ�����ߵ�·���� ���ֵ��û���㣬���������ʾ
            end
        end
    end
end
%%���ͼ������ϰ�
%Matrix = zeros(size(block));
 for i = 1:m
     for j=1:n
      if(block(i,j)==1)
         Matrix(i+1,j+1)=-inf;%  �������ܼ�ǽ��Ե��
      elseif(block(i,j)==0)
          Matrix(i+1,j+1)=inf;% 
      else
          Matrix(i+1,j+1)=inf;% 
      end
     end
 end

    subplot(1,3,1);
    plot(Spoint(1),Spoint(2),'r+');
%%Ѱ·
Matrix(Spoint(1),Spoint(2))=0;
Matrix(Epoint(1),Epoint(2))=inf;
G=Matrix;%����ֵG
F=Matrix;%F
openlist=Matrix;
closelist=Matrix;
parentx=Matrix;
parenty=Matrix;
openlist(Spoint(1),Spoint(2)) =0;
%closelist(Epoint(1),Epoint(2))=inf;
%��ͼ
for i = 1:n+2
    for j = 1:m+2
        k = Matrix(i,j);
        if(k == -inf)
            subplot(1,3,1);
            plot(i,j,'r.');
        elseif(k == inf)
            subplot(1,3,1);
            plot(i,j,'gh');
        else
            subplot(1,3,1);
            plot(i,j,'gh');
        end
        hold on
    end
end
title('A*');
axis([0 m+3 0 n+3]);
subplot(1,3,1);
plot(Epoint(1),Epoint(2),'b+');
subplot(1,3,1);
plot(Spoint(1),Spoint(2),'b+');

while(1)% ѭ���ҵ�·��
    num=inf;%��Ȼ����ֵ
    for p=1:m+2
        for q=1:n+2
            if(openlist(p,q)==0&&closelist(p,q)~=1)%����ڿ����б�û�ڹر��б���
                Outpoint=[p,q];%��һ�� ��ʼ��
                if(F(p,q)>=0&&num>F(p,q))% �ڿ����б���ѡ��һ����С��Fֵ�Ľڵ㣻num������С��f�Ƚϣ�����С��
                    num=F(p,q);
                    Nextpoint=[p,q];
                end
            end
        end
    end
    closelist(Nextpoint(1),Nextpoint(2))=1;%��ʼ�����ر��б�
    for i = 1:3
        for j = 1:3
            k = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);%��Χ8�����Gֵ �͵�ǰ���Gֵ
            if(i==2&&j==2|closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)==1)
                continue;%ǰ���Gֵ ����
            elseif (k == -inf)% �ϰ��� ���迼�ǣ������뵽 �ر��б������ڸ��迼��
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j) = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
                closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=1;
            elseif (k == inf)% ������������ֵ
                distance=((i-2)^2+(j-2)^2)^0.5;%�������ĵ㵽���ܵ�ľ��� 1��1.4
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1),Nextpoint(2))+distance;%���㵽 ��һ����G
                openlist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=0;%�����ĵ�ĳ�ȥ���ɴ�����򣬼��뵽�����б���
               % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;%ŷ����¾�����������
                H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϸ��ӵĶԽ�����������
                H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                H=sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);%���㵽 ��һ����H


                % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϼ򵥵ĶԽ��ߺ���
                
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)+H;%���㵽 ��һ����F ����ֵ
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);%��¼�õ�ĸ��ڵ��x����
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);%��¼�õ�ĸ��ڵ��������
            else distance=((i-2)^2+(j-2)^2)^0.5;%����õ��Ѿ��ڿ����б��ڵ� �� �Ƚϸ��º�fֵ
                if(k>(distance+G(Nextpoint(1),Nextpoint(2))))
                k=distance+G(Nextpoint(1),Nextpoint(2));%������º��gֵ��ԭ�ȵ�С����������˵����һ���߶���
               % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;  %ŷ����¾�����������
                H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϸ��ӵĶԽ�����������
                H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                H=sqrt(2)*10*H_diagonal+10*(H_straight-2*H_diagonal);


                 % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϼ򵥵ĶԽ��ߺ���
                
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=k+H;%���¸���Fֵ�����Ѹ��ڵ㻻�ɵ�ǰ�Ľڵ�
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
                end
            end
            if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)%�Ѱ��ս�ķŵ������б��ڡ������·�����
                 parentx(Epoint(1),Epoint(2))=Nextpoint(1);
                parenty(Epoint(1),Epoint(2))=Nextpoint(2);
                break;
            end
        end
        if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)%�Ѱ��ս�ķŵ������б��ڡ������ҷ�����
             parentx(Epoint(1),Epoint(2))=Nextpoint(1);
                parenty(Epoint(1),Epoint(2))=Nextpoint(2);
            break;
        end
    end
    if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)
         parentx(Epoint(1),Epoint(2))=Nextpoint(1);
                parenty(Epoint(1),Epoint(2))=Nextpoint(2);
        break;
    end
end
    P=[];
    s=1;
while(1)%ѭ���һ���·��
    if(num==inf)%û·��
        break;
    end
    subplot(1,3,1);
    plot(Epoint(1),Epoint(2),'b+');

    P(s,:)=Epoint;%��·���� β�ڵ���뵽·��������
    s=s+1;
%      pause(1);
    xx=Epoint(1);%����Ŀ��ڵ�� ���ڵ�
    Epoint(1)=parentx(Epoint(1),Epoint(2));
    Epoint(2)=parenty(xx,Epoint(2));
    if(parentx(Epoint(1),Epoint(2))==Spoint(1)&&parenty(Epoint(1),Epoint(2))==Spoint(2))%������ݵ���ʼ�ڵ� ���������
        subplot(1,3,1);
        plot(Epoint(1),Epoint(2),'b+');
        P(s,:)=Epoint;
        break;
    end
end
P(s+1,:)=Spoint;

toc
%end

%=====print path==============
PrintPath(P);


