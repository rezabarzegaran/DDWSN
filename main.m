%clear all
%close all
%clc
%% Variables
%Simulation
use_saved_data = false;
visualize = false;
%Drones
%N_bounds = [6, 8];
%N = round(N_bounds(1) + (N_bounds(2)-N_bounds(1))*rand(1));
N = 4;
%Obstacle density in percentage
O_density_range = [0,0];
%Obstacle Collision Bounds
O_bound = [0.5 1.5];
%Maximum and Minimum of X boundary
X_bound = [-4, 4];
%Maximum and Minimum of Y boundary
Y_bound = [-4, 4];
%Maximum and Minimum of Z boundary
Z_bound = [0, 4];
%Omega bounds
Omega_Bounds = [X_bound;Y_bound;Z_bound];
%grid size
grid_Unit_size = 10;
grid_Len = 1 / grid_Unit_size;
%Z deployment limit
H_bound = [1.5, 3.5];
%discrete time step
dt=0.01;
% Simulation time
Ittmax = 400;
% Collision Bounds
D_bound = [0.15 0.15];
% Communication Range Bounds
R_bound = [2.0 2.0];
%number of Lambda Points
N_lambda=4;
%prediction Steps
N_prediction = 10;
N_tesselation = 10;
%% Prepare Variables
%Prepare domain
xDomain = linspace(X_bound(1), X_bound(2), grid_Unit_size*(X_bound(2)-X_bound(1)));
yDomain = linspace(Y_bound(1), Y_bound(2), grid_Unit_size*(Y_bound(2)-Y_bound(1)));
last_tesselation_step = 0;
%Prepare O
[obstacles, occ_ratio] = initObstacles(Omega_Bounds,O_density_range,O_bound);
%Prepare D
D = rand(N);
D = D_bound(1) + (D_bound(2) - D_bound(1)) * D;
D = triu(D, 1) + triu(D, 1).' + diag(diag(D));
% Prepare R
R = rand(N);
R = R_bound(1) + (R_bound(2) - R_bound(1)) * R;
R = triu(R, 1) + triu(R, 1).' + diag(diag(R));
%% Initial Network
drone = initDrone(N,Omega_Bounds,obstacles,D,R);
event_Probability = eventProbability(xDomain,yDomain,grid_Len);
overall_area = overalArea(xDomain,yDomain,grid_Len);
displayMessage = sprintf('Obstacle density is %d and number of obstacles is %d .',occ_ratio, length(obstacles));
disp(displayMessage)
displayMessage = sprintf('The number of drones is %d .',N);
disp(displayMessage)
%% Simulation
for step=1:1:Ittmax
    t=step*dt;
    for n=1:N
        [drone(n).gilbert{step}, drone(n).dist{step}] = CreateGilbert(n,drone,R);
    end
    %Tesselation
    if mod(step,N_tesselation)== 1
        for n=1:N
            W(step).Drone{n} = [];
            W(step).DroneAccuracy{n} = [];
        end
        for i=1:size(xDomain,2)
            for j=1:size(yDomain,2)
                omega = [xDomain(i) yDomain(j)];
                currentAccuracy = [];
                for n=1:N
                    currentAccuracy(n) = sensingAccuracy(omega,drone(n).position{end},drone(n).s_param);               
                end
                [m, inx] = max(currentAccuracy);
                W(step).Drone{inx}(end+1,:)= omega;
                W(step).DroneAccuracy{inx}(end+1)= m;
            end
        end
        last_tesselation_step = step;
        for k =1:N
            if size(W(step).Drone{k},1) == 0
                    displayMessage = sprintf('Wrong Tesselation with drone %d .',k);
                    disp(displayMessage)
            end
        end
    end
    
    %Gradient Calculation
    if mod(step,N_prediction)== 1
        for n=1:N
            drone(n).gradient{step} = calcGradient(W(last_tesselation_step).Drone{n},drone(n), grid_Len, step);
            drone(n).target{step} = drone(n).position{end} + drone(n).gradient{step};
        end     
        % Calculate the path
         [results, lesserNeighborhood] = calcTraj(N_prediction,drone,dt,D,obstacles,R,H_bound);
         for n =1: N
             drone(n).lesserNeighbor{step} = lesserNeighborhood{n}.neighbor;
         end
    end

    %Simulation
    for n=1:N
        current_step = mod(step,N_prediction) + 1;
        drone(n).control{step} = sampleCurve(results(n).Lambda,current_step,N_prediction);
        [drone(n).position{step+1} , drone(n).velocity{step+1}]= DisDroneDynamics(drone(n).position{end},drone(n).velocity{end},drone(n).control{step},drone(n).v_bound, drone(n).a_bound,dt);
    end

    coverage_Performance_temp = 0;
    gilbert_edges_temp = 0;
    neighborhood_temp = 0;
    accuracy_temp = 0;
    disptextlesser = '';
    for n=1:N
        [performance, accuracy] = sensingPerformance(W(last_tesselation_step).Drone{n},drone(n).position{end},drone(n).s_param,grid_Len);
        indivitual_coverage_performance(step,n)= performance;
        coverage_Performance_temp = coverage_Performance_temp + performance ;
        accuracy_temp = accuracy_temp + accuracy;
        if(n~=1)
            gilbert_edges_temp = gilbert_edges_temp + length(drone(n).gilbert{end});
            neighborhood_temp = neighborhood_temp + length(drone(n).lesserNeighbor{end});
            disptextlesser = strcat(disptextlesser, num2str(length(drone(n).lesserNeighbor{end})));
        end

    end
    gilbert_edges(step)= gilbert_edges_temp;
    neighborhood_edges(step) = neighborhood_temp;
    sparsity_ratio(step) = neighborhood_temp / gilbert_edges_temp;
    coverage_performance(step) = coverage_Performance_temp;
    coverage_percentage(step) = coverage_Performance_temp / event_Probability;
    sensing_accuracy(step) = accuracy_temp;
    sensing_efficinecy(step) = accuracy_temp / overall_area;

    displayMessage = sprintf('Ittr = %d , Coverage percentage = %d , Acuracy percentage = %s, Sparsity ratio = %d , links = %s.',step, coverage_percentage(step), sensing_efficinecy(step), sparsity_ratio(step), disptextlesser);
    %displayMessage = sprintf('Ittr = %d , Sparsity ratio = %d , links = %s.',step, sparsity_ratio(step), disptextlesser);

    disp(displayMessage)

end
displayMessage = [];
current_step = [];
i = [];

displayMessage = sprintf('Obstacle density is %d and number of obstacles is %d .',occ_ratio, length(obstacles));
disp(displayMessage)
displayMessage = sprintf('The number of drones is %d .',N);
disp(displayMessage)
%filename = "TC";
%filename = strcat(filename,num2str(simloop));
%filename = strcat(filename,".mat");
%save(filename);
%save('TC.mat');




