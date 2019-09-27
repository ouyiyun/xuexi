%% Potential field algorithm
% seminar TU KL
% Author: Francisco J. Garcia R.

clc;close all;clear all;

% Input Map

% Conection matrix - define admisible movement of robot
conn=[1 1 1;
      1 2 1;
      1 1 1];
  
mapOriginal=im2bw(imread('Maps/a_map3.bmp')); % input map read from a bmp file.

resolutionX=100;
resolutionY=100;

mapResized=imresize(mapOriginal,[resolutionX resolutionY]);
map=mapResized; 
sz = size(map)

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

temp_img = (map==0).*0 + (map==1).*255 + (mapResized-map).*150;
image(temp_img);   

%colormap(gray(256))

% Inital pose
disp('select source in the image');
[x,y] = ginput(1);
xs = double(int32(x)); 
ys = double(int32(y));
[xs ys]

% Goal Pose
disp('select goal in the image');
[x,y] = ginput(1);
xg = double(int32(x)); 
yg = double(int32(y));
[xg yg]

if length(find(conn==2))~=1, error('no robot specified in connection matrix'); end

% plot goal and source
temp_img(ys, xs  ) = 160;
temp_img(yg, xg  ) = 160;

image(temp_img);

% Attractive potential
hold on;
[X,Y] = meshgrid(0:1:sz(1)-1, 0:1:sz(2)-1);

U_att = (1/2) .* ( sqrt((X-xg).^2 +(Y - yg).^2) ) ^2;

% Repulsive potential

% Compute for every pixel the distance to the nearest obstacle(non zero
% elements which after inversion corespondent to the obstacles)
[D, IDX] = bwdist(map);

max_sc = max(max(U_att))

Di = max_sc * 10; % Distance of influence of the object

for i = 1:sz(1)
    for j = 1:sz(2)
        rd = D(i,j);
        if (rd <= Di)
             U_rep(i,j)  = (1/2)*(rd - Di)^2  ;
        else
           U_rep(i,j)  = 0;
        end
    end
end

U_att = U_att/max(U_att(:));
U_rep = (1 - U_rep/max(U_rep(:)) );

subplot(2,2,1)
image(temp_img);
surf(X,Y,U_att)
title('Attraction field','color','b');

subplot(2,2,2)
image(temp_img);
surf(X,Y,U_rep);
title('Repulsion field','color','b');

k_s  = 2; % Scaling factor
subplot(2,2,3)
U_sum = U_rep/max(U_rep(:))+k_s*U_att/max(U_att(:));
surf(X,Y,U_sum);
title('Total field','color','b');

% Potential Field Path Planning

% Pose Matrix
pose = zeros(sz(1),sz(2));
pose(ys,xs) = 1;
pose(yg,xg) = 1;

subplot(2,2,4)
hold on

contourf(U_sum,15);
spy(pose,'*r');
title('Path','color','b');

x = xs; y = ys;

last = U_sum(y-1,x-1);

try

    while (x ~= xg)||(y ~= yg)
        dis =[  U_sum(y-1,x-1), U_sum(y-1,x),   U_sum(y-1,x+1);
                U_sum(y,x-1),   U_sum(y,x),     U_sum(y,x+1);
                U_sum(y+1,x-1), U_sum(y+1,x),   U_sum(y+1,x+1)];

        m = min(dis(:));
        [r,c] = find(dis == m);

        U_sum(y,x) = inf;

        y = y-2+r;
        x = x-2+c;

        pose(y,x) = 1;
    end 
catch
     
end

spy(pose,'.r');


