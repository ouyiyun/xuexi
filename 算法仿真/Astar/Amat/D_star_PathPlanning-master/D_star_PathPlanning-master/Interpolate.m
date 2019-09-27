function out = Interpolate(points, sampling, interpolationType)
    % s�u�y do r�wnoodleg�o�ciowego interpolowania punkt�w
    % interpolation types:
    % linear
    % cubic
    % spline
    
    

    if nargin < 2
        sampling = 1;
        interpolationType = 'cubic';
    elseif nargin < 3
        interpolationType = 'cubic';
    end
    
        % niekt�re punkty sie powtarzaj� 
    indicesToDelete = [];
    for i = 1:1:length(points(:,1))-1
        if w_distance(points(i,:), points(i+1,:)) < 0.01
            indicesToDelete = [indicesToDelete, i];
        end
    end
    
    points(indicesToDelete, :) = [];
    
    if length(points(:,1)) < 4
        out = points;
        return
    end
    
    len = length(points(:,1));
    distance = zeros(len,1);
    
    % wyznaczenie odleg�o��i pomi�dzy punktami
    for i = 2:1:len
        delta = w_distance(points(i,:), points(i-1,:));
        distance(i) =  delta + distance(i-1);
    end
    
    t = 0 : sampling : distance(end); 
    out = interp1(distance, points, t, interpolationType);
end

