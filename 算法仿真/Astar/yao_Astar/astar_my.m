function astar_my
n=20;                %20��դ��
wallpercent=0.15;    %�ϰ������ڵ�ͼ�ı���

[field, startposind, goalposind, costchart, fieldpointers]=initializeField(n,wallpercent);

setOpen=[startposind]; setOpenCosts=[0]; setOpenHeuristics = [Inf]; 
setClosed = []; setCloseCosts = [];
movementdirections ={'R', 'L', 'D', 'U'};

axishandle = creatFigure(field, costchart, startposind, goalposind); %����һ����ͼ
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen)
    
    [temp,ii]=min(setOpenCosts+setOpenHeuristics);
    [costs, heuristics, posinds] = findFValue(setOpen(ii),setOpenCosts(ii),field,goalposind);
    setClosed = [setClosed; setOpen(ii)];%����һ���¾���
    setCloseCosts = [setCloseCosts; setOpenCosts(ii)];
    
    if(ii>1 && ii<length(setOpen))  %�ӿ����б���ɾ��temp
        setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
        setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)]; %�����Ƿֺ� �ü�ֻ�����о��󣬲ü�һ�л���
        setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
        if size(setOpenCosts) ~= size(setOpenHeuristics)
            disp('temp locates in middle');
        end
    elseif (ii == 1)
        setOpen = setOpen(2:end);
        setOpenCosts = setOpenCosts(2:end);
        setOpenHeuristics = setOpenHeuristics(2:end);
        if size(setOpenCosts) ~= size(setOpenHeuristics)
            disp('temp locates in first');
        end
    else
        setOpen = setOpen(1:end-1);
        setOpenCosts = setOpenCosts(1:end-1);
        setOpenHeuristics = setOpenHeuristics(1:end-1);
        if size(setOpenCosts) ~= size(setOpenHeuristics)
            disp('temp locates in last');
        end
    end;
    for jj=1:length(posinds)
%         costs(jj)
        if ~isinf(costs(jj))%isinf ��ʾ�������Ƿ��н� �н�Ϊ0�� �޽�Ϊ1
%             disp('number has range');
%             [setClosed; setOpen]
            if ~max([setClosed; setOpen] == posinds(jj))%�ж���չ��Ľڵ��Ƿ��ڿ����б��ر��б��� (������չ�ĸ��ڵ��Ѿ�ɾ��) 
                fieldpointers(posinds(jj)) = movementdirections(jj);%movementdirections(jj) �Ǽ�ס���ڵ������ƶ��õ����ӽڵ�
                costchart(posinds(jj)) = costs(jj);%�������ֵ
                setOpen = [setOpen; posinds(jj)];%���ӽڵ�(��չ��)���뿪���б���
                setOpenCosts = [setOpenCosts; costs(jj)];
                setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];
            elseif max(setOpen == posinds(jj)) %��չ�ڵ��ڿ����б���
%                 disp('<<<<<<<<<<<<<<<<<<<2');
                I=find(setOpen == posinds(jj));%�����б����ҵ�����չ�ڵ��Сһ���Ľڵ��λ��
                if setOpenCosts(I) > costs(jj)
                  fieldpointers(setOpen(I)) = movementdirections(jj);
                  costchart(setOpen(I)) = costs(jj);
                  setOpenCosts(I) = costs(jj);
                  setOpenHeuristics(I) = heuristics(jj);
                end
            else  %��չ�ڵ��ڹر��б��в����κδ���
%                  disp('<<<<<<<<<<<<<<<<<<<3');
            end
        else
%             disp('asfd 1');
        end
    end
%     setOpen
    if isempty(setOpen) 
        break; 
    end
%     set(axishandle, 'CDdata', [costchart costchart(:,end), costchart(end,:) costchart(end, end)]);
%     set(gca, 'CLim', [0 1.1*max(costchart(find(costchart < Inf)))]);
    drawnow;
end

if max(ismember(setOpen, goalposind))
    disp('solution found');
    p = findWayBack(goalposind, fieldpointers);
%    plot(p(:,2), p(:,1), 'Color', 0.2*ones(3,1), 'LineWidth', 4);
    [m,n] = size(p); 
    ii=m;
    for i=1:m
         x_save(i) = p(ii,2)+0.5;
         y_save(i) = p(ii,1)+0.5;
         ii=ii-1;
    end;
    plot(x_save, y_save,'r' );
%��ȡ�˶���Ԫ
    save_motive_primity = textread('motive_primity_2.mprim');
    for i=1:12
        for j=1:2
            if j==1
                x_motive(i)=save_motive_primity(i*10,j); %�����˶���Ԫ���յ�
            else
                y_motive(i)=save_motive_primity(i*10,j);
            end;    
        end;
    end
    for i=1:120
        for j=1:2
            if j==1
               save_motion_x(i)=save_motive_primity(i,j);%�����˶���Ԫ��·����
            else
               save_motion_y(i)=save_motive_primity(i,j);
            end;    
        end;
    end
%��ȡ�˶���Ԫ

%��·�����Ϸ��˶���Ԫ
goal_x = x_save(m); goal_y = y_save(m); %����Ŀ���
count_loop = 0; sign = 1; min_sign = -1; 
reach_goal = 0; %�����յ��־λ
% interpolation_value = 0; %ƥ���·���� ��Ҫ��ֵ ��־Ϊ
save_motion_id = -1; %�ҵ��˶���Ԫ��λ��
test_i = 0; test_j = 0; j = 0; plan_obstacle = 0; 
for ii = sign:m %mΪ·�������
    pose1_x = x_save(sign); pose1_y = y_save(sign);
    save_motive_id_count = 1;
    %����
    save_motive_id  = [];
    for i_motive = 1:12 %12 һ����12���˶���Ԫ ���˶���Ԫ����
        %�ж����ɹ켣�����ܷ���ײ���ϰ���
        for i_pose = 1:10
            motion_x = save_motion_x(i_pose + i_motive * 10 - 10) + pose1_x;
            motion_y = save_motion_y(i_pose + i_motive * 10 - 10) + pose1_y;
                if fix(motion_x) ~= 0 &&  fix(motion_y) ~= 0 
                    if field(fix(motion_y), fix(motion_x)) == 1 ||  field(fix(motion_y), fix(motion_x)) == 0 %pose2_y + motion_x, pose2_x + motion_y
                        plan_obstacle = plan_obstacle + 1;
                    end
                end
        end
        %�ж����ɹ켣�����ܷ���ײ���ϰ���   
        if( plan_obstacle == 10) 
            save_motive_id(save_motive_id_count) = i_motive;
            save_motive_id_count = save_motive_id_count + 1;
        end
         plan_obstacle = 0;
    end
 
    if save_motive_id_count == 1
        disp('only one motive ');
        save_motive_id_count = 2;
    end
    
    
    for i_motive1 = 1:save_motive_id_count-1    %12 һ����12���˶���Ԫĩ������
        get_save_motive_id = save_motive_id(i_motive1);
        pose2_x = pose1_x + x_motive(get_save_motive_id); pose2_y = pose1_y + y_motive(get_save_motive_id);
        if pose2_x == goal_x && pose2_y == goal_y  % �ж��Ƿ�Ŀ���
            disp('reach goal 3');
            save_motion_id = get_save_motive_id;
            reach_goal = 1;
             for i_pose = 1:10
                 plot_x(i_pose + count_loop * 10) = pose1_x + save_motion_x(i_pose + save_motion_id * 10 - 10);
                 plot_y(i_pose + count_loop * 10) = pose1_y + save_motion_y(i_pose + save_motion_id * 10 - 10);
             end    
        else
            reach_goal = 0;
        end
        
        if reach_goal == 1
            break;
        end  
     end  %12 һ����12���˶���Ԫĩ������
    if reach_goal == 1
        break;
    end
    
    %����
    for i_motive1 = 1:save_motive_id_count-1
        get_save_motive_id = save_motive_id(i_motive1);
        pose2_x1 = x_motive(get_save_motive_id) + pose1_x;  pose2_y1 = y_motive(get_save_motive_id) + pose1_y; 
%         if reach_goal == 0  %δ���յ�
            for j = sign:m %ƥ��·����
                if pose2_x1 == x_save(j) &&  pose2_y1 == y_save(j) &&  min_sign < j% 
                    min_sign = j;
                    save_motion_id = get_save_motive_id; 
                    if save_motion_id == 0
                        disp('out wrong   1');
                    end
                end
            end
%         end
        sign = min_sign;
        for i_pose = 1:10
             plot_x(i_pose + count_loop * 10) = pose1_x + save_motion_x(i_pose + save_motion_id * 10 - 10);
             plot_y(i_pose + count_loop * 10) = pose1_y + save_motion_y(i_pose + save_motion_id * 10 - 10);
        end
    end %end of for
    count_loop = count_loop + 1;
end
    plot(plot_x, plot_y,'Color', 0.2*ones(3,1), 'LineWidth', 4);
    drawnow;
elseif isempty(setOpen)
    disp('No solution');
end

function p = findWayBack(goalposind, fieldpointers)
    disp('find way back');
    n = length(fieldpointers);
    posind = goalposind;
    [py px] = ind2sub([n n], posind);
    p = [py px];
    while ~strcmp(fieldpointers{posind}, 'S')
        switch fieldpointers{posind}
            case 'L'
                px = px - 1;
            case 'R'
                px = px + 1;
            case 'U'
                py = py -1;
            case 'D'
                py = py + 1;
        end
        p = [p; py px];
        posind = sub2ind([n n], py, px);
    end
    
function [field, startposind,goalposind, costchart, fieldpointers] = initializeField(n,wallpercent)
field = ones(n,n);
field(ceil(n^2.*rand(floor(n*n*wallpercent),1))) = Inf;
startposind =1;                         %��ʼ������
goalposind = 246;                       %�յ�����
costchart=NaN*ones(n,n);
costchart(startposind) = 0;
field(startposind)=0;
% field(goalposind)=0;
fieldpointers = cell(n,n);
fieldpointers{startposind}='S'; fieldpointers{goalposind}='G';
fieldpointers(field == Inf) = {0};

function axishandle = creatFigure(field, costchart, startposind, goalposind)
    if isempty(gcbf)
        f1=figure('Position',[600 100 500 500], 'Units', 'Normalized', 'MenuBar', 'none');
    else
        gcf; cla;
    end    
    n=length(field);
    field(field < Inf) = 0;
    pcolor([1:n+1],[1:n+1], [field field(:,end); field(end,:) field(end,end)]);
  
    %
%     if field(1.1, 1.1) == 0
%         disp('meeting ');
%     end
    %���Ժ��� field �������������
    
%     cmap=flipud(colormap('jet'));
%     cmap(1,:)=zeros(3,1); cmap(end,:)=ones(3,1);
%     colormap(flipud(cmap));
    hold on;
    axishandle = pcolor([1:n+1],[1:n+1],[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
    [goalposy, goalposx]=ind2sub([n,n], goalposind);
    [startposy, startposx]=ind2sub([n,n], startposind);
%     x=1:10;
%     plot(x,x);
    plot(goalposx+0.5,goalposy+0.5,'ys','MarkerSize',10,'LineWidth',6);%LineWidth ��־���С MarkerSize���־��Ĵ�С
    plot(startposx+0.5, startposy+0.5, 'go', 'MarkerSize',10, 'LineWidth', 6);
%     x=1:10;
%     hold on;
%     bb = textread('motive_primity_2.mprim');
%     for i=1:120
%     %     hold off;
%         for j=1:2
%             if j==1
%                 x_save(i)=bb(i,j);
%             else
%                 y_save(i)=bb(i,j);
%             end;    
%         end;
%         hold on;
%         if (rem(i, 10) == 0)
%             plot(x_save,y_save);
% %             plot(x_save+1,y_save+1);% ��һ��դ����ͨ��(1,1) ����(0.0)
%         end;
%     end;
%     hold on;
%     for i=1:21
%         for j=1:21
%             plot(x_save+i,y_save+j);
%         end;
%     end;
 function [cost, heuristic, posinds] = findFValue(posind,costsofar,field,goalind)
    n=length(field);
    [currentpos(1) currentpos(2)] = ind2sub([n,n], posind);
    [goalpos(1) goalpos(2)] = ind2sub([n,n], goalind);
    cost=Inf*ones(4,1); heuristic=Inf*ones(4,1); pos = ones(4,2);
    
%     cost=field(1,1)
%     cost=field(1,4)
%     if cost == Inf
%         disp('meeting osbtacle');
%              end
    
    newx=currentpos(2)-1; newy=currentpos(1);%��չ�����դ��
    if newx>0 %x���Ǵ���1�� (��߲���Խ��)
        pos(1,:)=[newy newx];
        heuristic(1)=abs(goalpos(2)-newx)+abs(goalpos(1)-newy);
        cost(1)=costsofar+field(newy, newx);
    end;
    newx=currentpos(2)+1; newy=currentpos(1);
    if(newx<=n)
        pos(2,:)=[newy newx];
        heuristic(2)=abs(goalpos(2)-newx)+abs(goalpos(1)-newy);
        cost(2)=costsofar+field(newy, newx);
    end;
    
    newx=currentpos(2); newy=currentpos(1)-1;
    if newy>0
        pos(3,:)=[newy newx];
        heuristic(3)=abs(goalpos(2)-newx)+abs(goalpos(1)-newy);
        cost(3)=costsofar+field(newy, newx);
    end;
    newx=currentpos(2); newy=currentpos(1)+1;
    if newy<=n
        pos(4,:)=[newy newx];
        heuristic(4)=abs(goalpos(2)-newx)+abs(goalpos(1)-newy);
        cost(4)=costsofar+field(newy, newx);
    end;
    
    posinds=sub2ind([n n], pos(:,1), pos(:,2));%ת��������
     

    
