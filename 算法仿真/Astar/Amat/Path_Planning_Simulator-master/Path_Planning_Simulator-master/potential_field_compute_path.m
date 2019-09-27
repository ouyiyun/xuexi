function [final_path, path_found] = potential_field_compute_path(map, mapOriginal, source, goal, display_process, resolutionX, resolutionY)
xg = goal(2);
yg = goal(1);

xs = source(2);
ys = source(1);

sz = size(map);

path_found = true;

% Attractive potential
hold on;
[X,Y] = meshgrid(0:1:sz(1)-1, 0:1:sz(2)-1);

U_att = (1/2) .* ( sqrt((X-xg).^2 +(Y - yg).^2) ) ^2;

% Repulsive potential

% Compute for every pixel the distance to the nearest obstacle(non zero
% elements which after inversion corespondent to the obstacles)
[D, IDX] = bwdist(map);

max_sc = max(max(U_att));

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

k_s  = 2; % Scaling factor
U_sum = U_rep/max(U_rep(:))+k_s*U_att/max(U_att(:));

% Potential Field Path Planning

% Pose Matrix
pose = zeros(sz(1),sz(2));
pose(ys,xs) = 1;
pose(yg,xg) = 1;

x = xs; y = ys;

last = U_sum(y-1,x-1);

counter = 1;

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
        
        final_path(counter, (1:2)) = [ y, x ];
        counter = counter +1;
    end 
catch


end
    
if(display_process == true)
    figure(2);
    colormap('jet')
    
    subplot(2,2,1)
    image(map);
    surf(X,Y,U_att)
    title('Attraction field');
    
    subplot(2,2,2)
    image(map);
    surf(X,Y,U_rep);
    title('Repulsion field');
    
    subplot(2,2,3)
    surf(X,Y,U_sum);
    title('Total field: rep + k * att');
    
    subplot(2,2,4)
    hold on

    contourf(U_sum,15);
    spy(pose,'*r');
    title('Computed path');
    
    spy(pose,'.r');
end

end