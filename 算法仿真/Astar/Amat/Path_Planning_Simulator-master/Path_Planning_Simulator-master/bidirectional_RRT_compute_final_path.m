function [final_path, is_path_Found] = bidirectional_RRT_compute_final_path(map, mapOriginal, source, goal, display_process, resolutionX, resolutionY)

% Parameters

% stepsize >= disTh
stepsize=3; % size of each step of the RRT
disTh=3; % nodes closer than this threshold are taken as almost the same

maxFailedAttempts = 10000;

temp_img = (map==0).*0 + (map==1).*255;

 % plot goal and source
 temp_img(goal(1), goal(2) ) = 160;
 temp_img(source(1), source(2) ) = 160;             

if(display_process == true)
    figure(2);
    colormap(gray(256))
    subplot(1,2,1);
    image(temp_img);
    rectangle('position',[1 1 size(map)-1],'edgecolor','k');
    title('RRT: Blue(source) - red(goal)');
end

RRTree1=double([source -1]); % First RRT rooted at the source, representation node and parent index
RRTree2=double([goal -1]); % Second RRT rooted at the goal, representation node and parent index
counter=0;
tree1ExpansionFail=false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail=false; % sets to true if expansion after set number of attempts fails

while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs
    
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
        if ~tree1ExpansionFail && isempty(pathFound) && display_process
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b');
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail]=rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from goal towards source
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end % path found
        if ~tree2ExpansionFail && isempty(pathFound) && display_process
            line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','r');
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~isempty(pathFound) % path found
         if display_process
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
            counter=counter+1;M(counter)=getframe;
        end
        final_path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3); % add nodes from RRT 1 first
        while prev>0
            final_path=[RRTree1(prev,1:2);final_path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev>0
            final_path=[final_path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
    
    if(display_process)
       %pause(0.5); 
    end
end

%%
if size(pathFound,1)<=0
    disp('no path found.'); 
    is_path_Found = false;
    %final_path(1, (1:2)) = [0 0];
    return
else
    is_path_Found = true;
end

pathLength=0;

for i=1:length(final_path)-1, pathLength=pathLength+distanceCost_RRT(final_path(i,1:2),final_path(i+1,1:2)); end

if(display_process == true)
    subplot(1,2,2);
    image(temp_img);
    rectangle('position',[1 1 size(map)-1],'edgecolor','k');
    line(final_path(:,2),final_path(:,1));
    
    title('Computed path');
end

end