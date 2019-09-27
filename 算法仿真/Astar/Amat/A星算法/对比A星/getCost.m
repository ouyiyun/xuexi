function cost = getCost(openList,goalPoint,n)
global point
cost = [];
for ii = 1:length(openList)
    cost = [cost,getGcost(openList(ii),n)+getHcost(openList(ii),goalPoint,n)];
end
end