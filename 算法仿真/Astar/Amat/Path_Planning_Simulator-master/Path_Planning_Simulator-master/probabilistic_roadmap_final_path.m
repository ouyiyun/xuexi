function [final_path, pathFound] = probabilistic_roadmap_final_path(map, mapOriginal, source, goal, display_process, resolutionX, resolutionY)

display = false; 
temp_img = (map==0).*0 + (map==1).*255;

if(display_process == true)
    figure(2)
    display = true; 
    colormap(gray(256))
    subplot(1,2,1);
        
    image(temp_img);
end    

max_iterations = 1000;
k=80; % number of points in the PRM

rectangle('position',[1 1 size(map)-1],'edgecolor','k')
vertex=[source;goal]; % source and goal taken as additional vertices in the path planning to ease planning of the robot
if display, rectangle('Position',[vertex(1,2)-5,vertex(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
if display, rectangle('Position',[vertex(2,2)-5,vertex(2,1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
tic;
while length(vertex)<k+2 % iteratively add vertices
    x=double(int32(rand(1,2) .* size(map)));
    if feasiblePoint(x,map), 
        vertex=[vertex;x]; 
        
        if display
            rectangle('Position',[x(2)-2,x(1)-2,4,4],'Curvature',[1,1],'FaceColor','r'); 
            %pause(0.5);
        end
    end
end

edges=cell(k+2,1); % edges to be stored as an adjacency list
for i=1:k+2
    for j=i+1:k+2
        if checkPath(vertex(i,:),vertex(j,:),map);
            edges{i}=[edges{i};j];edges{j}=[edges{j};i];
            
            if display
                line([vertex(i,2);vertex(j,2)],[vertex(i,1);vertex(j,1)]); 
            end
        end
    end
end


%structure of a node is taken as index of node in vertex, historic cost, heuristic cost, total cost, parent index in closed list (-1 for source) 
Q=[1 0 heuristic(vertex(1,:),goal) 0+heuristic(vertex(1,:),goal) -1]; % the processing queue of A* algorihtm, open list
closed=[]; % the closed list taken as a list
pathFound=false;
counter = 1;
while size(Q,1)>0
     [A, I]=min(Q,[],1);
     n=Q(I(4),:); % smallest cost element to process
     Q=[Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; % delete element under processing
     if n(1)==2 % goal test
         pathFound=true;break;
     end
     for mv=1:length(edges{n(1),1}) %iterate through all edges from the node
         newVertex=edges{n(1),1}(mv);
         if length(closed)==0 || length(find(closed(:,1)==newVertex))==0 % not already in closed
             historicCost=n(2)+historic(vertex(n(1),:),vertex(newVertex,:));
             heuristicCost=heuristic(vertex(newVertex,:),goal);
             totalCost=historicCost+heuristicCost;
             add=true; % not already in queue with better cost
             if length(find(Q(:,1)==newVertex))>=1
                 I=find(Q(:,1)==newVertex);
                 if Q(I,4)<totalCost, add=false;
                 else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                 end
             end
             if add
                 Q=[Q;newVertex historicCost heuristicCost totalCost size(closed,1)+1]; % add new nodes in queue
             end
         end           
     end
     closed=[closed;n]; % update closed lists
     counter = counter + 1;
     
     if( counter > max_iterations)
         pathFound=false;
         break;
     end
end

if ~pathFound
    disp('no path found')
    final_path(1, (1:2)) = [0 0];
end

final_path=[vertex(n(1),:)]; %retrieve path from parent information
prev=n(5);
while prev>0
    final_path=[vertex(closed(prev,1),:);final_path];
    prev=closed(prev,5);
end

if(display_process == true)
    subplot(1,2,2);
    
    temp_img = (map==0).*0 + (map==1).*255;
    image(temp_img);
    rectangle('position',[1 1 size(map)-1],'edgecolor','k')
    line(final_path(:,2),final_path(:,1),'color','r');
end


end