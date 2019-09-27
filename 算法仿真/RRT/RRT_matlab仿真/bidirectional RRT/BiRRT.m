clc 
clear all
close all

map=im2bw(imread('map5.bmp'));
maxK=1000;
stepsize=20;
disTh=19;
source=[50 160];
goal=[490 490];

if ~feasiblePoint(source,map)
    error('起始点无效，请重新输入！1 free 0 occupy！');
end

if ~feasiblePoint(goal,map)
    error('目标点无效，请重新输入！1 free 0 occupy！');
end

counter=0;
imshow(map);
hold on
plot([source(2);goal(2)], [source(1);goal(1)], 'k*');
counter=counter+1;M(counter)=getframe;

sourceTree=[source -1];
goalTree=[goal -1];
pathFound=[];
goalTreeExtendFail=false;
sourceTreeExtendFail=false;
% 若toExtend为真则拓展sourceTree，反之，则goalTree
toExtend=true;
counter=0;
% pause(10);

tic;
while isempty(pathFound)
    if goalTreeExtendFail && sourceTreeExtendFail
        break;
    end
    if toExtend
        [sourceTree, goalTree, pathFound, extendTreeFail,num]=extconTree(sourceTree, goalTree, goal, map, stepsize, disTh, maxK);
        if ~isempty(pathFound), tree=1; break;end
        sourceTreeExtendFail=extendTreeFail;
        if ~sourceTreeExtendFail, goalTreeExtendFail=false;end
        toExtend=~toExtend;
        plot([sourceTree(end,2);sourceTree(sourceTree(end,3),2)],[sourceTree(end,1);sourceTree(sourceTree(end,3),1)],'ro');
        line([sourceTree(end,2);sourceTree(sourceTree(end,3),2)],[sourceTree(end,1);sourceTree(sourceTree(end,3),1)],'color','b')
        k=1;
        while k<=num
            plot([goalTree(end-k+1,2);goalTree(goalTree(end-k+1,3),2)],[goalTree(end-k+1,1);goalTree(goalTree(end-k+1,3),1)],'ro');
            line([goalTree(end-k+1,2);goalTree(goalTree(end-k+1,3),2)],[goalTree(end-k+1,1);goalTree(goalTree(end-k+1,3),1)],'color','g');
            k=k+1;
        end
        counter=counter+1;M(counter)=getframe;
        continue
    else
        [goalTree, sourceTree, pathFound, extendTreeFail, num]=extconTree(goalTree, sourceTree, source, map, stepsize, disTh, maxK);
        if ~isempty(pathFound), tree=0;break;end
        goalTreeExtendFail=extendTreeFail;
        if ~goalTreeExtendFail, sourceTreeExtendFail=false;end
        toExtend=~toExtend;
        plot([goalTree(end,2);goalTree(goalTree(end,3),2)],[goalTree(end,1);goalTree(goalTree(end,3),1)],'ro');
        line([goalTree(end,2);goalTree(goalTree(end,3),2)],[goalTree(end,1);goalTree(goalTree(end,3),1)],'color','g')
        k=1;
        while k<=num
            plot([sourceTree(end-k+1,2);sourceTree(sourceTree(end-k+1,3),2)],[sourceTree(end-k+1,1);sourceTree(sourceTree(end-k+1,3),1)],'ro');
            line([sourceTree(end-k+1,2);sourceTree(sourceTree(end-k+1,3),2)],[sourceTree(end-k+1,1);sourceTree(sourceTree(end-k+1,3),1)],'color','b');
            k=k+1;
        end
        counter=counter+1;M(counter)=getframe;
        continue
    end
end

if goalTreeExtendFail && sourceTreeExtendFail
    error('无法找到有效路径');
end

if ~isempty(pathFound)
    path=[pathFound(1,1:2)];
    if tree
        prev=pathFound(1,3);
        while prev>0
            path=[sourceTree(prev,1:2);path];
            prev=sourceTree(prev,3);
        end
        prev=pathFound(1,4);
        while prev>0
            path=[path;goalTree(prev,1:2)];
            prev=goalTree(prev,3);
        end
    else
        prev=pathFound(1,4);
        while prev>0
            path=[sourceTree(prev,1:2);path];
            prev=sourceTree(prev,3);
        end
        prev=pathFound(1,3);
        while prev>0
            path=[path;goalTree(prev,1:2)];
            prev=goalTree(prev,3);
        end
    end
end

toc;
if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc-5,pathLength); 
imshow(map);
plot(pathFound(2),pathFound(1),'b*');
plot(path(1,2),path(1,1),'k*');
plot(path(end,2),path(end,1),'g*');
plot(path(2:end-1,2),path(2:end-1,1),'ro');
line(path(:,2),path(:,1));






















