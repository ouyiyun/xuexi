%% a* algorithm
% seminar TU KL
% Author: Francisco J. Garcia R.
% based on: 
%    R. Kala (2014) Code for Robot Path Planning using A* algorithm, 
%    Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

%% Load Map and initial parameters
figure(1)
mapOriginal=im2bw(imread('Maps/a_map2.bmp')); % input map read from a bmp file.

resolutionX=100;
resolutionY=100;

mapResized=imresize(mapOriginal,[resolutionX resolutionY]);
map=mapResized; 

% Conection matrix - define admisible movement of robot
conn=[1 1 1;
      1 2 1;
      1 1 1];

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

image((map==0).*0 + (map==1).*255 + (mapResized-map).*150);
colormap(gray(256))

disp('select source in the image');
[x,y] = ginput(1);
source=[double(int8(y)) double(int8(x))];    % source position in Y, X format

disp('select goal in the image');
[x,y] = ginput(1);
goal = [double(int8(y)) double(int8(x))];    % goal position in Y, X format

if length(find(conn==2))~=1, error('no robot specified in connection matrix'); end

%% Compute path

%structure of a node is taken as positionY, positionX, historic cost, heuristic cost, total cost, parent index in closed list (-1 for source) 
Q=[source 0 heuristic(source,goal) 0+heuristic(source,goal) -1]; % the processing queue of A* algorihtm, open list
closed=ones(size(map)); % the closed list taken as a hash map. 1=not visited, 0=visited
closedList=[]; % the closed list taken as a list
pathFound=false;
tic;
counter=0;

size(Q)

while size(Q,1)>0
     [A, I]=min(Q,[],1);
     n=Q(I(5),:); % smallest cost element to process
     Q=[Q(1:I(5)-1,:);Q(I(5)+1:end,:)]; % delete element under processing
     if n(1)==goal(1) && n(2)==goal(2) % goal test
         pathFound=true;break;
     end
     [rx,ry,rv]=find(conn==2); % robot position at the connection matrix
     [mx,my,mv]=find(conn==1); % array of possible moves
     for mxi=1:size(mx,1) %iterate through all moves
         newPos=[n(1)+mx(mxi)-rx n(2)+my(mxi)-ry]; % possible new node
         if checkPath(n(1:2),newPos,map) %if path from n to newPos is collission-free
              if closed(newPos(1),newPos(2))~=0 % not already in closed
                  historicCost=n(3)+historic(n(1:2),newPos);
                  heuristicCost=heuristic(newPos,goal);
                  totalCost=historicCost+heuristicCost;
                  add=true; % not already in queue with better cost
                  if length(find((Q(:,1)==newPos(1)) .* (Q(:,2)==newPos(2))))>=1
                      I=find((Q(:,1)==newPos(1)) .* (Q(:,2)==newPos(2)));
                      if Q(I,5)<totalCost, add=false;
                      else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                      end
                  end
                  if add
                      Q=[Q;newPos historicCost heuristicCost totalCost size(closedList,1)+1]; % add new nodes in queue
                  end
              end
         end           
     end
     closed(n(1),n(2))=0;closedList=[closedList;n]; % update closed lists
     i0 = counter;
     i1 = 40;
     counter=counter+1;
     
     if display_process == true && (rem(i0,i1) == 0) 
         temp_img = (map==0).*0 + ((closed==0).*(map==1)).*125 + ((closed==1).*(map==1)).*255 + (mapResized - map).*100 ;
         
         % plot goal and source
         temp_img(goal(1), goal(2) ) = 160;
            temp_img(source(1), source(2) ) = 160;
        
         image(temp_img);
       
        M(counter)=getframe;
     end
     
     size(Q)
end

if ~pathFound
    error('no path found')
end

%% Plot complete path
figure(2)
path=[n(1:2)]; %retrieve path from parent information
prev=n(6);
while prev>0
    path=[closedList(prev,1:2);path];
    prev=closedList(prev,6);
end
path=[(path(:,1)*size(mapOriginal,1))/resolutionX (path(:,2)*size(mapOriginal,2))/resolutionY];
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+historic(path(i,:),path(i+1,:)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength);

imshow(mapOriginal);
line(path(:,2),path(:,1));

