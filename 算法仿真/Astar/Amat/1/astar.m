%function astar(map,dataNode)
tic
%=============数据=====================
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
m = 10;n = 10;%表示不含四周墙的地图 行数  列数 
 Spoint = [2 2];% 开始点
%Spoint = [3 3];
Epoint = [m+1 n+1];%接受点
 %%//障碍表   
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
%========map 判空是否有效========
  [m n] = size(map);
 if ([m n] ~= [0 0])
  block =map;
  m = m;
  n = n;%表示不含四周墙的地图 行数  列数 
 else
     if([dataNode(2,1) dataNode(2,2)] > [m n])
        dataNode(2,:)= [m n]
     end
 end
 Spoint = dataNode(1,:)+1;% 开始点
 Epoint =dataNode(2,:)+1;%接受点
%%地图
for i = 1:m+2% 表示加了左右的墙
    if i == 1
        for j = 1:n+2
            Matrix(i,j) = -inf;%墙
        end
    elseif i == m+2
        for j = 1:n+2%墙
            Matrix(i,j) = -inf;
        end
    else
        for j = 1:n+2
            if ((j == 1)|(j == n+2))
                Matrix(i,j) = -inf;%墙
            else
                Matrix(i,j) = inf;%其他都可以 表示可以走的路，但 损耗值还没计算，先用无穷表示
            end
        end
    end
end
%%向地图添加障障碍
%Matrix = zeros(size(block));
 for i = 1:m
     for j=1:n
      if(block(i,j)==1)
         Matrix(i+1,j+1)=-inf;%  这是四周加墙的缘故
      elseif(block(i,j)==0)
          Matrix(i+1,j+1)=inf;% 
      else
          Matrix(i+1,j+1)=inf;% 
      end
     end
 end

    subplot(1,3,1);
    plot(Spoint(1),Spoint(2),'r+');
%%寻路
Matrix(Spoint(1),Spoint(2))=0;
Matrix(Epoint(1),Epoint(2))=inf;
G=Matrix;%计算值G
F=Matrix;%F
openlist=Matrix;
closelist=Matrix;
parentx=Matrix;
parenty=Matrix;
openlist(Spoint(1),Spoint(2)) =0;
%closelist(Epoint(1),Epoint(2))=inf;
%画图
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

while(1)% 循环找到路径
    num=inf;%当然消耗值
    for p=1:m+2
        for q=1:n+2
            if(openlist(p,q)==0&&closelist(p,q)~=1)%如果在开启列表并没在关闭列表中
                Outpoint=[p,q];%第一次 起始点
                if(F(p,q)>=0&&num>F(p,q))% 在开启列表内选择一个最小的F值的节点；num是与最小的f比较，找最小的
                    num=F(p,q);
                    Nextpoint=[p,q];
                end
            end
        end
    end
    closelist(Nextpoint(1),Nextpoint(2))=1;%起始点进入关闭列表
    for i = 1:3
        for j = 1:3
            k = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);%周围8个点的G值 和当前点的G值
            if(i==2&&j==2|closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)==1)
                continue;%前点的G值 跳过
            elseif (k == -inf)% 障碍区 不予考虑，并加入到 关闭列表内在于给予考略
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j) = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
                closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=1;
            elseif (k == inf)% 计算可走区域的值
                distance=((i-2)^2+(j-2)^2)^0.5;%计算中心点到四周点的距离 1，1.4
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1),Nextpoint(2))+distance;%计算到 下一步的G
                openlist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=0;%把中心点的除去不可达的区域，加入到开放列表内
               % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;%欧几里德距离启发函数
                H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较复杂的对角线启发函数
                H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                H=sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);%计算到 下一步的H


                % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较简单的对角线函数
                
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)+H;%计算到 下一步的F 消耗值
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);%记录该点的父节点的x坐标
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);%记录该点的父节点的有坐标
            else distance=((i-2)^2+(j-2)^2)^0.5;%如果该点已经在开放列表内的 化 比较更新后f值
                if(k>(distance+G(Nextpoint(1),Nextpoint(2))))
                k=distance+G(Nextpoint(1),Nextpoint(2));%如果更新后的g值比原先的小，更新它，说明这一步走对了
               % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;  %欧几里德距离启发函数
                H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较复杂的对角线启发函数
                H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                H=sqrt(2)*10*H_diagonal+10*(H_straight-2*H_diagonal);


                 % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较简单的对角线函数
                
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=k+H;%从新更新F值，并把父节点换成当前的节点
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
                end
            end
            if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)%已把终结的放到开发列表内。在上下方向上
                 parentx(Epoint(1),Epoint(2))=Nextpoint(1);
                parenty(Epoint(1),Epoint(2))=Nextpoint(2);
                break;
            end
        end
        if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)%已把终结的放到开发列表内。在左右方向上
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
while(1)%循环找回溯路径
    if(num==inf)%没路径
        break;
    end
    subplot(1,3,1);
    plot(Epoint(1),Epoint(2),'b+');

    P(s,:)=Epoint;%把路径的 尾节点加入到路径矩阵中
    s=s+1;
%      pause(1);
    xx=Epoint(1);%回溯目标节点的 父节点
    Epoint(1)=parentx(Epoint(1),Epoint(2));
    Epoint(2)=parenty(xx,Epoint(2));
    if(parentx(Epoint(1),Epoint(2))==Spoint(1)&&parenty(Epoint(1),Epoint(2))==Spoint(2))%如果回溯到开始节点 则结束回溯
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


