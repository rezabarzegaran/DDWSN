function [osbtacle, occ_ratio]= initObstacles(Omega_Bounds,O_density,O_bound)
%This function generates obstacle objects.
osbtacle = [];
O_max = O_density(2);
O_min = O_density(1);
V = (Omega_Bounds(1,2)-Omega_Bounds(1,1))*(Omega_Bounds(2,2)-Omega_Bounds(2,1))*(Omega_Bounds(3,2)-Omega_Bounds(3,1));
O_target = O_min + (O_max - O_min) * rand(1);
V_target = O_target * V;

V_occupied = 0;

inprogress = true;
n_o=1;
while inprogress
        distance= O_bound(1) + (O_bound(2) - O_bound(1)) * rand(1);
        v = 4 * pi * distance^3 / 3;
        x= Omega_Bounds(1,1) + (Omega_Bounds(1,2) - Omega_Bounds(1,1)) * rand(1);
        y= Omega_Bounds(2,1) + (Omega_Bounds(2,2) - Omega_Bounds(2,1)) * rand(1);
        z= Omega_Bounds(3,1) + (Omega_Bounds(3,2) - Omega_Bounds(3,1)) * rand(1);
        if(V_occupied + v) <= V_target
            V_occupied = V_occupied + v;
            osbtacle(n_o).distance = distance;
            osbtacle(n_o).x = x;
            osbtacle(n_o).y = y;
            osbtacle(n_o).z = z;
            n_o = n_o+1;

        else
            inprogress = false;
        end


end

occ_ratio = V_occupied / V;

end