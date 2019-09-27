% Potential field function simulation along with the steering controller
clear all; close all;

%% Setting up the environment

figure(1)
mapOriginal=im2bw(imread('Maps/a_map1.bmp')); % input map read from a bmp file.

resolutionX=100;
resolutionY=100;

mapResized=imresize(mapOriginal,[resolutionX resolutionY]);
si = size(mapOriginal);

% Environment(map) boundaries
posMinBound = [0 0];
posMaxBound = [si(1) si(2)];

% Start and end(goal) position
startPos = [-0.5 0.8];
endPos = [1.2 -0.8];

image((map==0).*0 + (map==1).*255 + (mapResized-map).*150);
colormap(gray(256))

disp('select source in the image');
[x,y] = ginput(1);
source=[double(int8(y)) double(int8(x))];    % source position in Y, X format

disp('select goal in the image');
[x,y] = ginput(1);
goal = [double(int8(y)) double(int8(x))];    % goal position in Y, X format

% Specifying the obstacle positions
obsPts = (map==0);
numObs = size(obsPts,2)/2

% Plotting the environment
%figure(1); clf; hold on;
%plotEnvironment(obsPts, posMinBound, posMaxBound, startPos, endPos);
%hold off

% Computing the centroids of the obstacles
for i=1:numObs
    obsCentroid(i,:)...
    = (obsPts(1,2*(i-1)+1:2*i) + obsPts(3,2*(i-1)+1:2*i))/2;
end

%% Setting up the potential field
Katt = 2;       % Scale of the attractive potential
Krep = 10;      % Scale of the repulsive potential
r0 = 0.24;      % Radius of repulsion from the closest point of obstacles
rc0 = 0.96;     % Raidus of repulsion from the centre of obstacles
Vmax = 10;      % Upper bound on potential
gVmax = 5;      % Upper bound on potential gradient
gVmin = -5;     % Lower bound on potential gradient
dx = 40;       % Grid size for x
dy = 40;       % Grid size for y

% Producing the coordinates of a rectangular grid (X,Y)
[X,Y] = meshgrid([posMinBound(1):dx:posMaxBound(1)],...
                 [posMinBound(2):dy:posMaxBound(2)]);
[n,m] = size(X);        % Size of the grid points
V = zeros(size(X));     % Potential field points
gV = zeros(2,n,m);      % Potential field gradients

%% Computing potential field/gradient at each grid point
for i=1:length(X(:,1))
    for j=1:length(Y(1,:))
        % Storing the position of the current grid point
        pos = [X(i,j) Y(i,j)];
        
        % Attractive potential computation
        V(i,j) = 1/2*Katt*norm(pos-endPos)^2;   % Potential
        gV(:,i,j) = Katt*(pos-endPos);          % Gradient
        
        % Repulsive potential computation
        for m=1:numObs
            curObs = obsPts(:,2*(m-1)+1:2*m);
            % If the grid point falls in the obstacles
            if (inpolygon(pos(1),pos(2),curObs(:,1),curObs(:,2)))
                V(i,j) = Vmax;          % Set the potential as the maximum
                gV(:,i,j) = [NaN NaN];
            % Otherwise (if the grid point is outside the obstacles)
            else
                % Potential based on minimum distance to obstacles
                curPoly = [curObs curObs([2:end, 1],:)];
                [minD, minPt, d, pt, ind] = minDistToEdges(pos, curPoly);
                if (minD < r0)
                    V(i,j) = V(i,j) + 1/2*Krep*(1/minD-1/r0)^2;
                    gV(:,i,j) = gV(:,i,j) +...
                                Krep*(-1/minD+1/r0)*(pos'-minPt')/minD^(3);
                end
                % Add potential of distance to center, to avoid getting
                % stuck on flat walls
                centD = norm(pos-obsCentroid(m,:));
                if (centD < rc0)
                    V(i,j) =  V(i,j) + 1/2*Krep*(1/centD-1/rc0)^2;
                    gV(:,i,j) = gV(:,i,j) +...
                                Krep*(-1/centD+1/rc0)*...
                                (pos'-obsCentroid(m,:)')/...
                                centD^(3);
                end
            end
        end
        V(i,j) = max(0, min(V(i,j), Vmax));
        gV(1,i,j) = max(gVmin, min(gV(1,i,j), gVmax));
        gV(2,i,j) = max(gVmin, min(gV(2,i,j), gVmax));
    end
end

%% Simulating a path down the gradient descent
Tmax = 10000;           % Maximum time step 
x = zeros(2,Tmax);      % State vectors [x y]
x(:,1) = startPos';     % Initial position
dx = 1;              % Position increment
t = 1;                  % Initial time step
gVcur = [1 1];          % Current grid point
while ((norm(gVcur)>0.01) && (t<Tmax))
    t = t+1;
    pos = x(:,t-1)';
    gVcur = Katt*(pos-endPos);
    for m=1:numObs
        curObs = obsPts(:,2*(m-1)+1:2*m);
        if (inpolygon(pos(1),pos(2),curObs(:,1),curObs(:,2)))
            gVcur = [NaN NaN];
        else
            curPoly = [curObs curObs([2:end, 1],:)];
            [minD,minPt, d, pt, ind] = minDistToEdges(pos, curPoly);
            if (minD < r0)
                gVcur = gVcur + Krep*(-1/minD+1/r0)*(pos-minPt)/minD^(3);
            end
            centD = norm(pos-obsCentroid(m,:));
            if (centD < rc0)
                gVcur = gVcur + Krep*(-1/centD+1/rc0)*(pos-obsCentroid(m,:))/centD^(3);
            end
        end
    end
    x(:,t) = x(:,t-1)-dx.*gVcur';   % Gradient descent implementation
end


%% Graph the path found
figure(1); hold on;
plot(x(1,1:t), x(2,1:t), 'g*');
hold on
%plot(exp_x,exp_y)
h_leg=legend('Obstacle','Obstacle','Planned Path', ...
             'Actual Robot Path');
set(h_leg,'FontSize',10);
title('Simulation vs Experimental Navigation of Potential Field Map');



% Graph the gradient descent
figure(2); clf; hold on
surf(X,Y,V);
title('Potential Field Gradient Map');