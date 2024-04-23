function drone= initDrone(N,O_Omega_Bounds,obstacles,D,R)
% This function generates drone objects.
    %Omega_Bounds(1,:) = O_Omega_Bounds(1,:) + [+1 -1];
    %Omega_Bounds(2,:) = O_Omega_Bounds(2,:) + [+1 -1];
    %Omega_Bounds(3,:) = O_Omega_Bounds(3,:) + [+1 -1];
    Omega_Bounds(1,:) = [-2 1];
    Omega_Bounds(2,:) = [-2  1];
    Omega_Bounds(3,:) = [2 3];
    for n=1:N
        drone(n).velocity{1}=[0 0 0];
        %drone(n).g0=[0 0 0];
        drone(n).control{1} = [0,0,0];
        drone(n).gradient{1} = [0, 0, 0];
        drone(n).v_bound=[-1 1];
        drone(n).a_bound=[-2 2];
        %angle = round(5+40*rand(1));
        angle = round(5+25*rand(1));
        %height = 1.5 + 2.5 * rand(1);
        height = 1.5 + 2 * rand(1);
        %hrate=5+20 * rand(1);
        hrate=5+15 * rand(1);
        %arate=0.5+8 * rand(1);
        arate=0.5+5 * rand(1);
        %angle = 20;
        %height = 2.5;
        %hrate = 10;
        %arate = 3;
        drone(n).s_param=[15;height;0.9;0;40;3;180;angle];
        drone(n).gilbert{1}=[];
        drone(n).dist{1}=[];
        drone(n).lesserNeighbor{1} = [];
        goodpalcement=false;
        while (~goodpalcement)
            drone(n).position{1}(1)=Omega_Bounds(1,1) + (Omega_Bounds(1,2)-Omega_Bounds(1,1))*rand(1);
            drone(n).position{1}(2)=Omega_Bounds(2,1) + (Omega_Bounds(2,2)-Omega_Bounds(2,1))*rand(1);
            drone(n).position{1}(3)=Omega_Bounds(3,1) + (Omega_Bounds(3,2)-Omega_Bounds(3,1))*rand(1);
            obstacleCollision = obstacleCheck(drone(n).position{1},obstacles);
            droneCollision = droneCollisionCheck(drone,D);
            dronenetwork = droneNetworkCheck(drone,R);
            goodpalcement = (~obstacleCollision) && (~droneCollision) && dronenetwork;
            
        end

    end

    end

function collision = obstacleCheck(p,obstacles)
collision = false;
for i=1:size(obstacles)
    O_center = [obstacles(i).x,obstacles(i).y,obstacles(i).z];
    distance = norm(p-O_center);
    if(distance<=obstacles(i).distance)
        collision = true;
    end
                    
end

end

function collision = droneCollisionCheck(drone,D)
    collision = false;
    lastindex = length(drone);
    for i = 1:lastindex-1
        distance = norm(drone(lastindex).position{1}-drone(i).position{1});
        if(distance<=D(lastindex,i))
            collision = true;
        end
    end

end

function connected = droneNetworkCheck(drone,R)
    connected = false;
    lastindex = length(drone);
    for i = 1:lastindex-1
        distance = norm(drone(lastindex).position{1}-drone(i).position{1});
        if(distance<=0.7*R(lastindex,i))
            connected = true;
        end
    end
    if(lastindex == 1)
        connected = true;
    end

end