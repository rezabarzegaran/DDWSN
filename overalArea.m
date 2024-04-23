function overal_Area = overalArea(xDomain,yDomain, grid_Len)
% This function is used for calculating the deployment area.
% Then furtherused in sensitivity.
overal_Area = 0;
    for i=1:size(xDomain,2)
            for j=1:size(yDomain,2)
                area = grid_Len * grid_Len;
                overal_Area = overal_Area + area;
            end
    end
end