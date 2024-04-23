function [performance, accuracy] = sensingPerformance(W,p,s_Param,grid_Len)
% This function calculates the sensing accuracy and performance
performance = 0;
accuracy = 0;
    for i = 1:size(W,1)
        omega = W(i,:);
        f = probabDist(omega(1),omega(2));
        area = grid_Len * grid_Len;
        prob = f * area;
        acc = sensingAccuracy(omega,p,s_Param);
        performance = performance + acc*prob;
        accuracy = accuracy + acc*area;
    end
end