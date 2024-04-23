function performance = eventProbability(xDomain,yDomain,grid_Len)
% This is the sensing probality function C.
performance = 0;
    for i=1:size(xDomain,2)
            for j=1:size(yDomain,2)
                omega = [xDomain(i) yDomain(j)];
                f = probabDist(omega(1),omega(2));
                prob = f * grid_Len * grid_Len;
                performance = performance + prob;
            end
    end
end