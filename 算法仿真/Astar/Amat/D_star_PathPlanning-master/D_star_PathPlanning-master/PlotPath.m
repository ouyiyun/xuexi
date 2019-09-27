function out = PlotPath(state, scalling, mapName)
% uses B-spline interpolation    
    path = state.path';
    path(:,3) = 1;
    path(:,4) = 1;

    if nargin <3
        scalling = 1;
        imag = state.map;
    else
        tmp = LoadMap(  strcat(mapName, '.png'), 1);
        imag = tmp.map;
    end

    
    if scalling ~= 1
        path(:,1) = path(:,1)-2.5;
        path(:,2) = path(:,2)-2.5;
        pathInterpolated = BSpline(path*scalling);
    else
        pathInterpolated = path;
    end
    %%5
    for it = pathInterpolated'
        a = round(it);
        imag(a(1), a(2)) = 0.6;
    end
    figure(100)
    out.handle = imshow(imag);
    out.path = pathInterpolated;
    
end