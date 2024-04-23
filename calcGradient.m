function gradient = calcGradient(W,drone, grid_Len, step)
%This function calculates the gradient ascend.
p = drone.position{end};
% Here a learning rate must be defined. Examples are provided
%learning_rate = 10-0.01*step;
learning_rate = 1;
if(learning_rate <= 1)
    learning_rate = 1;
end
s_Param = drone.s_param;
[c0, temp] = sensingPerformance(W,p,s_Param,grid_Len);

p1x = p + [grid_Len , 0 , 0];
[c1x, temp] = sensingPerformance(W,p1x,s_Param,grid_Len);
gradient_x = (c1x - c0) / grid_Len;

p1y = p + [0 , grid_Len , 0];
[c1y, temp] = sensingPerformance(W,p1y,s_Param,grid_Len);
gradient_y = (c1y - c0) / grid_Len;


p1z = p + [0 , 0 , grid_Len];
[c1z, temp] = sensingPerformance(W,p1z,s_Param,grid_Len);
gradient_z = (c1z - c0) / grid_Len;

gradient = [gradient_x, gradient_y, gradient_z];
gradient = learning_rate * gradient;
end