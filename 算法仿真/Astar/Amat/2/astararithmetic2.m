function [astar_path,cost_astar,astar_time]=astararithmetic2(Astar_coor,nodmat,obstaclenumber,noox,nooy,nooz,s,d)
tic;
astar_path=[];

f_value_start=max([abs(nodmat(d,2)-nodmat(s,2)),abs(nodmat(d,3)-nodmat(s,3)),abs(nodmat(d,4)-nodmat(s,4))]);          %��ʼԴ�ڵ��f����
second_value(s)=abs(nodmat(d,2)-nodmat(s,2))+abs(nodmat(d,3)-nodmat(s,3))+abs(nodmat(d,4)-nodmat(s,4)); %��������ֵ
node(s).message=[f_value_start,second_value(s),s,nodmat(s,2),nodmat(s,3),nodmat(s,4),0];
openlist=[node(s).message]; %��ʼ��openlist
closedlist=[0 0 0 0 0 0 0]; %��ʼ��closedlist
cost_astar=0;
node(s).pionter=[s];  %ָ���ʼ��

%% *************************************************************************



while (isempty(find(closedlist(:,3)==d,1)) && ~isempty(openlist))
%%
    cost_astar=cost_astar+1;
    if isempty(openlist)
        disp('erorr,openlist is empty');
        return
    end  %�ж�open�б��Ƿ�Ϊ�գ�Ϊ��������ʧ��
  %%%%%%%%%%%%%%  
    openlist=sortrows(openlist); %��openlist�еĽڵ㰴f����ֵ������������
    n=openlist(1,3);             %nΪopenlist��f������С�Ľڵ�ı��
    closedlist=[closedlist;openlist(1,:)]; %��openlist��f������С�Ľڵ����closedlist��
    if n==d
        astar_path=node(n).pionter;
        break
    end
        
    openlist=setdiff(openlist,openlist(1,:),'rows'); %��openlist��f������С�Ľڵ��openlist��ɾ��
   %%%%%%%%%%%%%%%%%%% 
    
    node(n).succeed=node_expand(n,noox,nooy,nooz);  %������̽ڵ�
    
    node(n).succeed=setdiff(node(n).succeed,obstaclenumber);     %�ų��ϰ��ڵ�
    
    for i=1:length(node(n).succeed)
        node(node(n).succeed(i)).pionter_t=[node(n).pionter,node(n).succeed(i)];  %Ϊÿ����̽ڵ�������ʱָ��
    
        g_value(node(n).succeed(i))=length(node(node(n).succeed(i)).pionter_t);  %��̽ڵ��g����ֵ
        
        if find(openlist(:,3)==node(n).succeed(i))       %�����̽ڵ�����openlist����
            j=find(openlist(:,3)==node(n).succeed(i));   %�ҳ��ڵ���openlist�����е�λ��
            if openlist(j,7)>g_value(node(n).succeed(i))  %�����̵�g���۱�ԭʼ·����g����С
                openlist(j,1)=openlist(j,1)-(openlist(j,7)-g_value(node(n).succeed(i)));  %����openlist��f����
                openlist(j,7)=g_value(node(n).succeed(i));  %����openlist��g����
                node(node(n).succeed(i)).pionter=node(node(n).succeed(i)).pionter_t;  %���´˽ڵ��ָ��
            end
            
        elseif find(closedlist(:,3)==node(n).succeed(i))       %�����̽ڵ�����closedlist����
               j=find(closedlist(:,3)==node(n).succeed(i));   %�ҳ��ڵ���closedlist�����е�λ��
               if closedlist(j,7)>g_value(node(n).succeed(i))  %�����̵�g���۱�ԭʼ·����g����С
                  closedlist(j,1)=closedlist(j,1)-(closedlist(j,7)-g_value(node(n).succeed(i)));  %����closedlist��f����
                  closedlist(j,7)=g_value(node(n).succeed(i));  %����closedlist��g����
                  node(node(n).succeed(i)).pionter=node(node(n).succeed(i)).pionter_t;  %���´˽ڵ��ָ��
               end 
            
        else
            node(node(n).succeed(i)).pionter=node(node(n).succeed(i)).pionter_t;%��̽ڵ�Ϊ�½ڵ㣬����ʱָ���Ϊ����ָ��
            h_value(node(n).succeed(i))=max([abs(nodmat(d,2)-nodmat(node(n).succeed(i),2)),abs(nodmat(d,3)-nodmat(node(n).succeed(i),3)),abs(nodmat(d,4)-nodmat(node(n).succeed(i),4))]); %��̽ڵ��h����ֵ
            f_value(node(n).succeed(i))=g_value(node(n).succeed(i))+h_value(node(n).succeed(i));%��̽ڵ��f����ֵ
    
            second_value(node(n).succeed(i))=abs(nodmat(d,2)-nodmat(node(n).succeed(i),2))+abs(nodmat(d,3)-nodmat(node(n).succeed(i),3))+abs(nodmat(d,4)-nodmat(node(n).succeed(i),4));  %��̽ڵ�ĸ�������ֵ
        
            node(node(n).succeed(i)).message=[f_value(node(n).succeed(i)),second_value(node(n).succeed(i)),node(n).succeed(i),nodmat(node(n).succeed(i),2),nodmat(node(n).succeed(i),3),nodmat(node(n).succeed(i),4),g_value(node(n).succeed(i))]; %ÿ����̽ڵ���Ϣ
            openlist=[openlist;node(node(n).succeed(i)).message];  %����̽ڵ���ӵ�openlist��    
        end
        %%%%%%%%
        
        
    end   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
astar_path=node(d).pionter;
toc;
astar_time=toc;
return