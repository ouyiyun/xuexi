function astardemo_Hong
%ASTARDEMO Demonstration of ASTAR algorithm
%
%   Copyright Bob L. Sturm, Ph. D., Assistant Professor
%   Department of Architecture, Design and Media Technology
%     formerly Medialogy
%   Aalborg University i Ballerup
%     formerly Aalborg University Copenhagen
%   $Revision: 0.1 $  $Date: 2011 Jan. 15 18h24:24$

n = 20;   % field size n x n tiles  20*20�Ľ���
wallpercent = 0.15;  % this percent of field is walls   45%�Ľ�����Ϊ�谭�ǽ��

% create the n x n FIELD with wallpercent walls containing movement costs, 
% a starting position STARTPOSIND, a goal position GOALPOSIND, the costs 
% A star will compute movement cost for each tile COSTCHART, 
% and a matrix in which to store the pointers FIELDPOINTERS
[field, startposind, goalposind, costchart, fieldpointers] = ...
  initializeField(n,wallpercent);   %��ʼ������

% initialize the OPEN and CLOSED sets and their costs
setOpen = [startposind]; setOpenCosts = [0]; setOpenHeuristics = [Inf];
setClosed = []; setClosedCosts = [];
movementdirections = {'R','L','D','U','DR','DL','UR','UL'};

% keep track of the number of iterations to exit gracefully if no solution
counterIterations = 1;

% create figure so we can witness the magic
axishandle = createFigure(field,costchart,startposind,goalposind);

% as long as we have not found the goal or run out of spaces to explore
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen)  %ismember(A,B)������Aͬ��С�ľ�������Ԫ��1��ʾA����Ӧλ�õ�Ԫ����B��Ҳ���֣�0����û�г���
  % for the element in OPEN with the smallest cost
  [temp, ii] = min(setOpenCosts + setOpenHeuristics);   %��OPEN����ѡ�񻨷���͵ĵ�temp,ii�����±�(Ҳ���Ǳ������)
  % find costs and heuristic of moving to neighbor spaces to goal
  % in order 'R','L','D','U'
  [costs,heuristics,posinds] = findFValue(setOpen(ii),setOpenCosts(ii), ...
    field,goalposind,'euclidean');       %��չtemp���ĸ�����㣬���������posinds������������ʵ�ʴ���costs����������heuristics
  % put node in CLOSED and record its cost
  setClosed = [setClosed; setOpen(ii)];     %��temp����CLOSE����
  setClosedCosts = [setClosedCosts; setOpenCosts(ii)];  %��temp�Ļ��Ѽ���ClosedCosts
  % update OPEN and their associated costs  ����OPEN�� ��Ϊ�������
  if (ii > 1 && ii < length(setOpen))   %temp��OPEN����м䣬ɾ��temp
    setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
    setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)];
    setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
  elseif (ii == 1)
    setOpen = setOpen(2:end);   %temp��OPEN��ĵ�һ��Ԫ�أ�ɾ��temp
    setOpenCosts = setOpenCosts(2:end);
    setOpenHeuristics = setOpenHeuristics(2:end);
  else     %temp��OPEN������һ��Ԫ�أ�ɾ��temp
    setOpen = setOpen(1:end-1);
    setOpenCosts = setOpenCosts(1:end-1);
    setOpenHeuristics = setOpenHeuristics(1:end-1);
  end
  % for each of these neighbor spaces, assign costs and pointers; 
  % and if some are in the CLOSED set and their costs are smaller, 
  % update their costs and pointers
  for jj=1:length(posinds)      %������չ���ĸ����������
%       aa = length(posinds);
    % if cost infinite, then it's a wall, so ignore
    if ~isinf(costs(jj))    %����˵��ʵ�ʴ��۲�ΪInf,Ҳ����û������ǽ
      % if node is not in OPEN or CLOSED then insert into costchart and 
      % movement pointers, and put node in OPEN
      if ~max([setClosed; setOpen] == posinds(jj)) %����˵㲻��OPEN���CLOSE����
        fieldpointers(posinds(jj)) = movementdirections(jj); %���˵�ķ�����ڶ�Ӧ��fieldpointers��
        aa =  posinds(jj);
        costchart(posinds(jj)) = costs(jj); %��ʵ�ʴ���ֵ�����Ӧ��costchart��
        setOpen = [setOpen; posinds(jj)]; %���˵����OPEN����
        setOpenCosts = [setOpenCosts; costs(jj)];   %����OPEN��ʵ�ʴ���
        setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];    %����OPEN����������
      % else node has already been seen, so check to see if we have
      % found a better route to it.
      elseif max(setOpen == posinds(jj)) %����˵���OPEN����
        I = find(setOpen == posinds(jj));   %�ҵ��˵���OPEN���е�λ��
        % update if we have a better route
        if setOpenCosts(I) > costs(jj)  %�����OPEN���еĴ˵�ʵ�ʴ��۱��������õĴ�
          costchart(setOpen(I)) = costs(jj);    %����ǰ�Ĵ��۴���costchart�У�ע��˵���costchart�е�������������������һ�µģ�setOpen(I)��ʵ����posinds(jj)������ͬfieldpointers
          setOpenCosts(I) = costs(jj);      %����OPEN���еĴ˵���ۣ�ע��˵���setOpenCosts�е���������setOpen����һ�µģ���ͬsetOpenHeuristics
          setOpenHeuristics(I) = heuristics(jj);    %����OPEN���еĴ˵���������(����Ϊ�����û�б��)
          fieldpointers(setOpen(I)) = movementdirections(jj);   %���´˵�ķ���   
        end
      % else node has already been CLOSED, so check to see if we have
      % found a better route to it.
      else   %����˵���CLOSE���У�˵���Ѿ���չ���˵�
        % find relevant node in CLOSED
        I = find(setClosed == posinds(jj)); 
        % update if we have a better route
%         if setClosedCosts(I) > costs(jj)   %�����CLOSE���еĴ˵�ʵ�ʴ��۱��������õĴ���һ�����⣬�����˵���չ�ĵ㻹��Ҫ���µ�ǰ������!!!��
%           costchart(setClosed(I)) = costs(jj);  %����ǰ�Ĵ��۴���costchart��
%           setClosedCosts(I) = costs(jj);    %����CLOSE���еĴ˵����
%           fieldpointers(setClosed(I)) = movementdirections(jj); %���´˵�ķ���
%           cc = posinds(jj);
%         end                                                    
      end
    end
  end
  if isempty(setOpen) break; end        %��OPEN��Ϊ�գ�������Ծ��������е��Ѿ���ѯ���
  set(axishandle,'CData',[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
  % hack to make image look right
  set(gca,'CLim',[0 1.1*max(costchart(find(costchart < Inf)))]);    %CLim��CData�е�ֵ��colormap��Ӧ����: [cmin cmax] Color axis limits   ��������̫����ΪʲôҪ*1.1��
  drawnow;              %cmin is the value of the data mapped to the first color in the colormap. cmax is the value of the data mapped to the last color in the colormap
end

if max(ismember(setOpen,goalposind))    %���ҵ�Ŀ���ʱ
  disp('Solution found!');  %disp�� Display array�� disp(X)ֱ�ӽ�������ʾ����������ʾ�����֣����XΪstring����ֱ���������X
  % now find the way back using FIELDPOINTERS, starting from goal position
  p = findWayBack(goalposind,fieldpointers);
  dd = length(fieldpointers);
  dd
  % plot final path
  plot(p(:,2)+0.5,p(:,1)+0.5,'Color',0.2*ones(3,1),'LineWidth',4);
  drawnow;
elseif isempty(setOpen)
  disp('No Solution!'); 
end
% end of the main function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p = findWayBack(goalposind,fieldpointers)
% This function will follow the pointers from the goal position to the
% starting position
    disp('find way back');
    n = length(fieldpointers);  % length of the field
    posind = goalposind;
    % convert linear index into [row column]
    [py,px] = ind2sub([n,n],posind);
    % store initial position
    p = [py px];

    % until we are at the starting position
    while ~strcmp(fieldpointers{posind},'S')    %����ѯ���ĵ㲻��'S'���ʱ
      switch fieldpointers{posind}
        case 'L' % move left  �����øõ����Դ�㷽��Ϊ��ʱ
          px = px - 1;
        case 'R' % move right
          px = px + 1;
        case 'U' % move up
          py = py - 1;
        case 'D' % move down
          py = py + 1;
      end
      p = [p; py px];
      % convert [row column] to linear index
      posind = sub2ind([n n],py,px);
    end
% end of this function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [cost,heuristic,posinds] = findFValue(posind,costsofar,field, ...
  goalind,heuristicmethod)
% This function finds the movement COST for each tile surrounding POSIND in
% FIELD, returns their position indices POSINDS. They are ordered: right,
% left, down, up.
    n = length(field);  % length of the field
    % convert linear index into [row column]
    [currentpos(1) currentpos(2)] = ind2sub([n n],posind);  %��õ�ǰ����������꣬ע��currentpos(1)�������꣬currentpos(2)��������
    [goalpos(1) goalpos(2)] = ind2sub([n n],goalind);       %���Ŀ������������
    % places to store movement cost value and position
    cost = Inf*ones(4,1); heuristic = Inf*ones(4,1); pos = ones(4,2);  
    
    % if we can look left, we move from the right  �����ѯ����ô���Ǵ��ұ���
    newx = currentpos(2) - 1; newy = currentpos(1); 
    if newx > 0  %���û�е��߽�
      pos(1,:) = [newy newx];   %����µ�����
      switch lower(heuristicmethod)
        case 'euclidean'    %ŷ����þ���(���񰡣���)
          heuristic(1) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);    %heuristic(1)Ϊ������������ľ������
        case 'taxicab'
          heuristic(1) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);  
      end
      cost(1) = costsofar + field(newy,newx);   %costsofarΪ֮ǰ���ѵĴ��ۣ�field(newy,newx)Ϊ������в���ۣ�cost(1)Ϊ�����˷�������ʵ����
    end

    % if we can look right, we move from the left  ���Ҳ�ѯ
    newx = currentpos(2) + 1; newy = currentpos(1);
    if newx <= n
      pos(2,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(2) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(2) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
      end
      cost(2) = costsofar + field(newy,newx);
    end

    % if we can look up, we move from down  ���ϲ�ѯ
    newx = currentpos(2); newy = currentpos(1)-1;
    if newy > 0
      pos(3,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(3) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(3) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
      end
      cost(3) = costsofar + field(newy,newx);
    end

    % if we can look down, we move from up  ���²�ѯ
    newx = currentpos(2); newy = currentpos(1)+1;
    if newy <= n
      pos(4,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(4) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(4) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
      end
      cost(4) = costsofar + field(newy,newx);
    end
    
    % return [row column] to linear index
    posinds = sub2ind([n n],pos(:,1),pos(:,2)); %posindsΪ�˵���չ���ĸ������ϵ�����
% end of this function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%��ʼ������
function [field, startposind, goalposind, costchart, fieldpointers] = ...
  initializeField(n,wallpercent) 
% This function will create a field with movement costs and walls, a start
% and goal position at random, a matrix in which the algorithm will store 
% f values, and a cell matrix in which it will store pointers
    % create the field and place walls with infinite cost  ��ʼ�������ǽ
    field = ones(n,n) + 10*rand(n,n);   
    % field(ind2sub([n n],ceil(n^2.*rand(floor(n*n*wallpercent),1)))) = Inf;  %floor(x)��ȡ��������ȥ��С�������������ceil(x)��ȡ������������С�������������Inf����������
    field(ceil(n^2.*rand(floor(n*n*wallpercent),1))) = Inf; %ind2sub����������������(����λ�����)תΪ��ά����(�������е�����)�ģ�������ʵ����תΪ��ά����Ϳ��ԣ�����field���Է�����������
    % create random start position and goal position  ���ѡ��������Ϊ������յ�
%     startposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));   %sub2ind��������������ת��Ϊ�������꣬�����Ǳ�Ҫ�ģ���Ϊ�����startposind���ó�[x,y]����ʽ������field([x,y])��ʱ��
%     goalposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));    %�������Ƿ���x��y��Ԫ�أ����Ƿ�����������Ϊx��y������Ԫ��
    startposind = 1;   %sub2ind��������������ת��Ϊ�������꣬�����Ǳ�Ҫ�ģ���Ϊ�����startposind���ó�[x,y]����ʽ������field([x,y])��ʱ��
    goalposind = 22;    %�������Ƿ���x��y��Ԫ�أ����Ƿ�����������Ϊx��y������Ԫ��

% force movement cost at start and goal positions to not be walls ����ʼ��������Ϊ0�������Ϊǽ
    field(startposind) = 0; field(goalposind) = 0;  
    % put not a numbers (NaN) in cost chart so A* knows where to look  
    costchart = NaN*ones(n,n);      %costchart�����洢�������ʵ�ʴ��ۣ�NaN���������ݣ�����ȷ�Ĳ�����
    % set the cost at the starting position to be 0
    costchart(startposind) = 0;     %����ʵ�ʴ���
    % make fieldpointers as a cell array  ����n*n��Ԫ��
    fieldpointers = cell(n,n);      %fieldpointers�����洢���������Դ����
    % set the start pointer to be "S" for start, "G" for goal   �������Ϊ"S",�յ�����Ϊ"G"
    fieldpointers{startposind} = 'S'; fieldpointers{goalposind} = 'G';
    % everywhere there is a wall, put a 0 so it is not considered   ǽ����Ϊ0
    fieldpointers(field == Inf) = {0};      %�ܺõķ�ʽ��field == Inf ����ǽ��λ�ã�fieldpointers(field == Inf)������Ӧ��λ��
% end of this function

%%%%%%%%%%%%%%%%%%%%  
function axishandle = createFigure(field,costchart,startposind,goalposind)
% This function creates a pretty figure
    % If there is no figure open, then create one
    if isempty(gcbf)    %gcbf�ǵ�ǰ����ͼ��ľ��
      f1 = figure('Position',[450 150 500 500],'Units','Normalized', ...  
        'MenuBar','none');  %�����Position����ֵΪһ����Ԫ���� rect = [left, bottom, width, height]����һ������������ʾ����λ�ã����Ǵ���Ļ�����½Ǽ����
                            %normalized �� Units map the lower-left corner of the figure window to (0,0) and the upper-right corner to (1.0,1.0). 
%       Caxes2 = axes('position', [0.01 0.01 0.98 0.98],'FontSize',12, ...
%         'FontName','Helvetica');    %position����ǰ��figure���õĵ�λ��in normalized units where (0,0) is the lower-left corner and (1.0,1.0) is the upper-right
    else
      % get the current figure, and clear it ��õ�ǰͼ�����
      gcf; cla;
    end
    n = length(field);
    % plot field where walls are black, and everything else is white 0�Ǻ�ɫ
    field(field < Inf) = 0; %ע�⣬��Ȼ�޸���field�����������field���ھֲ�����������û��Ӱ���������е�field
    pcolor([1:n+1],[1:n+1],[field field(:,end); field(end,:) field(end,end)]); %����һ��һ��
    % set the colormap for the ploting the cost and looking really nice
%     cmap = flipud(colormap('jet')); %flipud���ڷ�ת���� colormapΪ����jet���͵���ɫ�� jet ranges from blue to red
    % make first entry be white, and last be black
%     cmap(1,:) = zeros(3,1); cmap(end,:) = ones(3,1); %�ı���ɫ��βɫ��Ϊ(0,0,0)�Ǻ�ɫ����ɫ��Ϊ(1,1,1)�ǰ�ɫ
    % apply the colormap, but make red be closer to goal  ��ɫ�Ǹ��ӽ�Ŀ�����ɫ
%     colormap(flipud(cmap));
    % keep the plot so we can plot over it
    
    %********���÷�ת�Ϳ���*********%
%     cmap = colormap('jet'); 
%     cmap(1,:) = ones(3,1); cmap(end,:) = zeros(3,1);
%     colormap(cmap);
    %*******************************%
    
    hold on;
    % now plot the f values for all tiles evaluated
    axishandle = pcolor([1:n+1],[1:n+1],[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
    % plot goal as a yellow square, and start as a green circle
    [goalposy,goalposx] = ind2sub([n,n],goalposind);    %ע�ⷵ�ص��к��е�λ��
    [startposy,startposx] = ind2sub([n,n],startposind);
    plot(goalposx+0.5,goalposy+0.5,'ys','MarkerSize',10,'LineWidth',6);     %��0.5��Ϊ�˰������Ƶ��������룬'ys'��y��ʾyellow,s��ʾSquare(����)
    plot(startposx+0.5,startposy+0.5,'go','MarkerSize',10,'LineWidth',6);   %'go'��g��ʾgreen,o��ʾCircle(Բ��)
    % add a button so that can re-do the demonstration
    uicontrol('Style','pushbutton','String','RE-DO', 'FontSize',12, ...
      'Position', [1 1 60 40], 'Callback','astardemo');
 % end of this function 
