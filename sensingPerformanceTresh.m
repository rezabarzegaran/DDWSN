function [performance, accuracy] = sensingPerformanceTresh(W,p,s_Param,grid_Len, tresh)
% This function is used to determine if a point is sensed or not
performance = 0;
accuracy = 0;
    for i = 1:size(W,1)
        omega = W(i,:);
        f = probabDist(omega(1),omega(2));
        area = grid_Len * grid_Len;
        prob = f * area;
        acc = sensingAccuracy(omega,p,s_Param);
        performance = performance + acc*prob;
        if(acc >= tresh)
            acc= 1;
        else
            acc = 0;
        end
        accuracy = accuracy + acc*area;
    end
end