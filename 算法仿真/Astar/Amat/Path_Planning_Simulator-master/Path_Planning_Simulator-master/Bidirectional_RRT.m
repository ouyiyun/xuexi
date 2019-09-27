%% Bidirectional Rapidly-exploring Random Trees algorithm
% seminar TU KL
% Author: Francisco J. Garcia R.
% based on: 
%   R. Kala (2014) Code for Robot Path Planning using Bidirectional Rapidly-exploring Random Trees, 
%   Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
close all
clear all
clc

%% Load Map and initial parameters
figure(1)
mapOriginal=im2bw(imread('Maps/a_map1.bmp')); % input map read from a bmp file.

resolutionX=500;
resolutionY=500;

mapResized=imresize(mapOriginal,[resolutionX resolutionY]);
map=mapResized; 

% Parameters
stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
display=true; % display of RRT

display_process=true; % display processing of nodes

% grow boundary by 1 unit pixel - take into account size of robot
for i=1:size(mapResized,1)
    for j=1:size(mapResized,2)
        if mapResized(i,j)==0
            if i-1>=1, map(i-1,j)=0; end
            if j-1>=1, map(i,j-1)=0; end
            if i+1<=size(map,1), map(i+1,j)=0; end
            if j+1<=size(map,2), map(i,j+1)=0; end
            if i-1>=1 && j-1>=1, map(i-1,j-1)=0; end
            if i-1>=1 && j+1<=size(map,2), map(i-1,j+1)=0; end
            if i+1<=size(map,1) && j-1>=1, map(i+1,j-1)=0; end
            if i+1<=size(map,1) && j+1<=size(map,2), map(i+1,j+1)=0; end
        end
    end
end

hold on
colormap(gray(256))

temp_img = (map==0).*0 + (map==1).*255 + (mapResized-map).*150;
image(temp_img);   

% Inital pose
disp('select source in the image');
[x,y] = ginput(1);
xs = double(int32(x)); 
ys = double(int32(y));
source = [ys xs ]

% Goal Pose
disp('select goal in the image');
[x,y] = ginput(1);
xg = double(int32(x)); 
yg = double(int32(y));
goal = [yg xg ]

% plot goal and source
temp_img(ys, xs ) = 160;
temp_img(yg, xg ) = 160;

image(temp_img);

%%%%% parameters end here %%%%%

tic;
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end

RRTree1=double([source -1]); % First RRT rooted at the source, representation node and parent index
RRTree2=double([goal -1]); % Second RRT rooted at the goal, representation node and parent index
counter=0;
tree1ExpansionFail=false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail=false; % sets to true if expansion after set number of attempts fails

while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
        if ~tree1ExpansionFail && isempty(pathFound) && display
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b');
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail]=rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from goal towards source
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end % path found
        if ~tree2ExpansionFail && isempty(pathFound) && display
            line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','r');
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~isempty(pathFound) % path found
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3); % add nodes from RRT 1 first
        while prev>0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev>0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
end

%%
if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost_RRT(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 

figure(2)
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));
