%% ¡¤
function routPlot(goalNum,n)
global point
ii = point(goalNum).num;
rout= [ii];
if ~isempty(point(goalNum).father)
    while point(ii).father ~= point(ii).num
        rout=[point(ii).father,rout];
        ii = point(ii).father;
    end
    disp('Solution Done!')
    disp(rout);
else
    disp('No Solution!')
end
[y,x]= ind2sub([n,n],rout);
plot(x-0.5,y-0.5,'LineWidth',2,'color','r')