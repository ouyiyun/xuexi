function [image, image_original, map, mapOriginal] = LoadMap(index, resolutionX, resolutionY)
    path_map = '';
    
    switch index
        case 1
            path_map = 'Maps/a_map1.bmp';
        case 2
            path_map = 'Maps/a_map2.bmp';
        case 3
            path_map = 'Maps/a_map3.bmp';
        case 4
            path_map = 'Maps/a_map4.bmp';
        case 5
            path_map = 'Maps/a_map5.bmp';
    end
    
    mapOriginal=im2bw(imread(path_map)); % input map read from a bmp file.

    mapResized=imresize(mapOriginal,[resolutionX resolutionY]);
    map=mapResized; 

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

    image =  (map==0).*0 + (map==1).*255 + (mapResized-map).*150;
    image_original = (map==0).*0 + (map==1).*255 + (mapResized-map).*255;
end