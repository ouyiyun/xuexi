clc;
clear;
n=50;
starNum=1;%起点位置
% starNum = randi(n*n,[1,1]);
goalNum=2500;%终点位置
%goalNum = randi(n*n,[1,1]);
banper=0.4;%障碍物比例
%% map init
figure('name','A*','NumberTitle','off','MenuBar','none');
global point 
for ii=1:n*n
    point(ii).num = ii;
    point(ii).father=[];
    point(ii).Gcost=[];
    point(ii).Hcost=[];
end
%% banper
banList=[randi(n*n,[1,floor(banper*n*n)])];
%load banList
banList(find(banList==goalNum))=[];
for jj = 1:length(banList)
    if banList(jj)~=goalNum || banList(jj)~=starNum
        point(banList(jj)).Gcost = Inf;
    end
end
point(starNum).Gcost=0;
point(starNum).father = point(starNum).num;
point(starNum).Hcost=getHcost(point(starNum),point(goalNum),n);

%% A*core
openList = [];
closeList = [];
closeListNum=[];
openListNum=[];
openList = [openList,point(starNum)];
while length(openList)
    % opneList
    costList = getCost(openList,point(goalNum),n);
    currentPoint = openList(find(costList==min(costList),1));
    openList(find(min(costList)==costList,1))=[];
    closeList = [closeList,currentPoint];
    neighbourNum = getNeighbour(currentPoint,n);
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
    for ii = 1:length(neighbourNum)
        if neighbourNum(ii)==point(goalNum).num
            point(neighbourNum(ii)).father = currentPoint.num;
            point(goalNum).father = currentPoint.num;
            disp('ok')
            routPlot(goalNum,n);
            return;
        end
            log1=0;
            try
                tmp=point(neighbourNum(ii)).Gcost;
                if tmp ==inf
                    log1 = 1;
                end
            catch
                log1=0;
            end
            if log1 || ismember(neighbourNum(ii),closeListNum)
                continue;
            elseif (ismember(neighbourNum(ii),openListNum))
            oldGcost = getGcost(point(neighbourNum(ii)),n);
            father = point(neighbourNum(ii)).father;
            point(neighbourNum(ii)).father = currentPoint.num;
            newGcost = getGcost(point(neighbourNum(ii)),n);
            if newGcost>oldGcost
                point(neighbourNum(ii)).father = father;
            else
                point(neighbourNum(ii)).Gcost = newGcost;
            end
            continue;
        elseif ~ismember(neighbourNum(ii),closeListNum)
            point(neighbourNum(ii)).father = currentPoint.num;
            point(neighbourNum(ii)).Gcost = getGcost(point(neighbourNum(ii)),n);
            point(neighbourNum(ii)).Hcost = getHcost(point(neighbourNum(ii)),point(goalNum),n);
            openList = [openList,point(neighbourNum(ii))];
            end
    end
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
    pause(0.1);
    mydrawnow(starNum,goalNum,banList,closeListNum,openListNum,n);
end