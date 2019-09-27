function [astar_path,cost_astar,astar_time]=astararithmetic2(Astar_coor,nodmat,obstaclenumber,noox,nooy,nooz,s,d)
tic;
astar_path=[];

f_value_start=max([abs(nodmat(d,2)-nodmat(s,2)),abs(nodmat(d,3)-nodmat(s,3)),abs(nodmat(d,4)-nodmat(s,4))]);          %初始源节点的f代价
second_value(s)=abs(nodmat(d,2)-nodmat(s,2))+abs(nodmat(d,3)-nodmat(s,3))+abs(nodmat(d,4)-nodmat(s,4)); %附属代价值
node(s).message=[f_value_start,second_value(s),s,nodmat(s,2),nodmat(s,3),nodmat(s,4),0];
openlist=[node(s).message]; %初始化openlist
closedlist=[0 0 0 0 0 0 0]; %初始化closedlist
cost_astar=0;
node(s).pionter=[s];  %指针初始化

%% *************************************************************************



while (isempty(find(closedlist(:,3)==d,1)) && ~isempty(openlist))
%%
    cost_astar=cost_astar+1;
    if isempty(openlist)
        disp('erorr,openlist is empty');
        return
    end  %判断open列表是否为空，为空则搜索失败
  %%%%%%%%%%%%%%  
    openlist=sortrows(openlist); %将openlist中的节点按f代价值进行升序排列
    n=openlist(1,3);             %n为openlist中f代价最小的节点的编号
    closedlist=[closedlist;openlist(1,:)]; %将openlist中f代价最小的节点放入closedlist中
    if n==d
        astar_path=node(n).pionter;
        break
    end
        
    openlist=setdiff(openlist,openlist(1,:),'rows'); %将openlist中f代价最小的节点从openlist中删除
   %%%%%%%%%%%%%%%%%%% 
    
    node(n).succeed=node_expand(n,noox,nooy,nooz);  %产生后继节点
    
    node(n).succeed=setdiff(node(n).succeed,obstaclenumber);     %排除障碍节点
    
    for i=1:length(node(n).succeed)
        node(node(n).succeed(i)).pionter_t=[node(n).pionter,node(n).succeed(i)];  %为每个后继节点生成临时指针
    
        g_value(node(n).succeed(i))=length(node(node(n).succeed(i)).pionter_t);  %后继节点的g代价值
        
        if find(openlist(:,3)==node(n).succeed(i))       %如果后继节点已在openlist表中
            j=find(openlist(:,3)==node(n).succeed(i));   %找出节点中openlist表中行的位置
            if openlist(j,7)>g_value(node(n).succeed(i))  %如果后继的g代价比原始路径的g代价小
                openlist(j,1)=openlist(j,1)-(openlist(j,7)-g_value(node(n).succeed(i)));  %更新openlist中f代价
                openlist(j,7)=g_value(node(n).succeed(i));  %更新openlist中g代价
                node(node(n).succeed(i)).pionter=node(node(n).succeed(i)).pionter_t;  %更新此节点的指针
            end
            
        elseif find(closedlist(:,3)==node(n).succeed(i))       %如果后继节点已在closedlist表中
               j=find(closedlist(:,3)==node(n).succeed(i));   %找出节点中closedlist表中行的位置
               if closedlist(j,7)>g_value(node(n).succeed(i))  %如果后继的g代价比原始路径的g代价小
                  closedlist(j,1)=closedlist(j,1)-(closedlist(j,7)-g_value(node(n).succeed(i)));  %更新closedlist中f代价
                  closedlist(j,7)=g_value(node(n).succeed(i));  %更新closedlist中g代价
                  node(node(n).succeed(i)).pionter=node(node(n).succeed(i)).pionter_t;  %更新此节点的指针
               end 
            
        else
            node(node(n).succeed(i)).pionter=node(node(n).succeed(i)).pionter_t;%后继节点为新节点，将临时指针变为永久指针
            h_value(node(n).succeed(i))=max([abs(nodmat(d,2)-nodmat(node(n).succeed(i),2)),abs(nodmat(d,3)-nodmat(node(n).succeed(i),3)),abs(nodmat(d,4)-nodmat(node(n).succeed(i),4))]); %后继节点的h代价值
            f_value(node(n).succeed(i))=g_value(node(n).succeed(i))+h_value(node(n).succeed(i));%后继节点的f代价值
    
            second_value(node(n).succeed(i))=abs(nodmat(d,2)-nodmat(node(n).succeed(i),2))+abs(nodmat(d,3)-nodmat(node(n).succeed(i),3))+abs(nodmat(d,4)-nodmat(node(n).succeed(i),4));  %后继节点的附属代价值
        
            node(node(n).succeed(i)).message=[f_value(node(n).succeed(i)),second_value(node(n).succeed(i)),node(n).succeed(i),nodmat(node(n).succeed(i),2),nodmat(node(n).succeed(i),3),nodmat(node(n).succeed(i),4),g_value(node(n).succeed(i))]; %每个后继节点信息
            openlist=[openlist;node(node(n).succeed(i)).message];  %将后继节点添加到openlist中    
        end
        %%%%%%%%
        
        
    end   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
astar_path=node(d).pionter;
toc;
astar_time=toc;
return