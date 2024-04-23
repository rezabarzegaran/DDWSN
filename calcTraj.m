function [results, LesserNeighborHood] = calcTraj(N_prediction,drone,dt,D,obstacles,R, H_bound)
% This function is the QP problem.
%Variables
N_last_step = 2;
N_lambda = 4;
N = length(drone);
% Create lambda variables
for n=1:N
    nodeName(n,:) = strcat('drone',int2str(n));
    Traj{n}.X = optimvar(nodeName(n,:),N_lambda+1,3,'LowerBound',-5,'UpperBound',5);
end


% Create sampled cruve
for n=1:N
    for step=1:N_prediction+1
        l = [0 , 0 ,0];
        for m=1:N_lambda+1          
            l = l + Traj{n}.X(m,:)*B(m-1,N_lambda,step-1,N_prediction);
        end
        Traj{n}.C(step,1)= l(1);
        Traj{n}.C(step,2)= l(2);
        Traj{n}.C(step,3)= l(3);
    end
end

% Create Prediction states
for n=1:N
    count = 1;
    for i=N_prediction + 1 - N_last_step:N_prediction+1
        p_temp = NewPredictDroneDynamics(drone(n).position{end},drone(n).velocity{end},Traj{n}.C,dt,i);
        Traj{n}.P(count,1)= p_temp(1);
        Traj{n}.P(count,2)= p_temp(2);
        Traj{n}.P(count,3)= p_temp(3);
        count = count +1;
    end
end

% Create objective
objec = 0;
for n=1:N
    for i = 1:N_last_step
        objec = objec + norm(Traj{n}.P(i,:)-drone(n).target{end})^2;
        %objec = objec + norm(drone{n}.C(i,:)-p_hat(n,:))^2;
    end
end

prob = optimproblem('Objective',objec);

%% Constraints
total_constriants = 0;
% initail point of the curve must be uqual to the current position
for n=1:N
    % for x
    constraintName = "Const" + num2str(total_constriants);
    prob.Constraints.(constraintName) = Traj{n}.X(1,1) == drone(n).position{end}(1);
    total_constriants = total_constriants + 1;
    % for y
    constraintName = "Const" + num2str(total_constriants);
    prob.Constraints.(constraintName) = Traj{n}.X(1,2) == drone(n).position{end}(2);
    total_constriants = total_constriants + 1;
    % for z    
    constraintName = "Const" + num2str(total_constriants);
    prob.Constraints.(constraintName) = Traj{n}.X(1,3) == drone(n).position{end}(3);
    total_constriants = total_constriants + 1;
end
% Curve velocity must remain within the limits
for n=1:N
    
end

% Curve acceleration must remain within the limits
for n=1:N

end

% Drones' predicted positions must remain within the deployment altitude
for n=1:N
    % for x
    for m=1:N_lambda+1
    % for z    
    constraintName = "Const" + num2str(total_constriants);
    prob.Constraints.(constraintName) = Traj{n}.X(m,3) >=H_bound(1);
    total_constriants = total_constriants + 1;
        constraintName = "Const" + num2str(total_constriants);
    prob.Constraints.(constraintName) = Traj{n}.X(m,3) <=H_bound(2);
    total_constriants = total_constriants + 1;
    end
end

% Collision avoidance
for n=1:N
    for m=1:N
        if(m~=n)
            for k=1:(N_lambda+1)
                Coll_X(k,1) = Traj{n}.X(k,1) - Traj{m}.X(k,1);
                Coll_X(k,2) = Traj{n}.X(k,2) - Traj{m}.X(k,2);
                Coll_X(k,3) = Traj{n}.X(k,3) - Traj{m}.X(k,3);
                for l=1:(N_lambda+1)
                    Coll_Y(l,1) = Traj{n}.X(l,1) - Traj{m}.X(l,1);
                    Coll_Y(l,2) = Traj{n}.X(l,2) - Traj{m}.X(l,2);
                    Coll_Y(l,3) = Traj{n}.X(l,3) - Traj{m}.X(l,3);
                    Coll_X_dot_Y = Coll_X(k,1)*Coll_Y(l,1) + Coll_X(k,2)*Coll_Y(l,2) + Coll_X(k,3)*Coll_Y(l,3);
                    constraintName = "Const" + num2str(total_constriants);
                    prob.Constraints.(constraintName) = Coll_X_dot_Y >= D(n,m)^2;
                    total_constriants = total_constriants + 1;
                end
            end

        end
    end
end


% Domain constraints
for n=1:N
    for o=1:length(obstacles)
        for k=1:(N_lambda+1)
            Coll_X(k,1) = Traj{n}.X(k,1) - obstacles(o).x;
            Coll_X(k,2) = Traj{n}.X(k,2) - obstacles(o).y;
            Coll_X(k,3) = Traj{n}.X(k,3) - obstacles(o).z;
            for l=1:(N_lambda+1)
                Coll_Y(l,1) = Traj{n}.X(l,1) - obstacles(o).x;
                Coll_Y(l,2) = Traj{n}.X(l,2) - obstacles(o).y;
                Coll_Y(l,3) = Traj{n}.X(l,3) - obstacles(o).z;
                Coll_X_dot_Y = Coll_X(k,1)*Coll_Y(l,1) + Coll_X(k,2)*Coll_Y(l,2) + Coll_X(k,3)*Coll_Y(l,3);
                constraintName = "Const" + num2str(total_constriants);
                prob.Constraints.(constraintName) = Coll_X_dot_Y >= obstacles(o).distance^2;
                total_constriants = total_constriants + 1;
            end
        end
    end
end

% Connectivity Constraints
LesserNeighborHood{1}.neighbor = NaN;
for n=2:N
    ind = calcLesserNeighbor(n,drone);
    %ind = 1:(n-1);
    LesserNeighborHood{n}.neighbor = ind;
    %ind = [];
    if(length(ind) < 1)
        displayMessage = sprintf('Drone %d disconnected in step %d',n, step);
        disp(displayMessage);
    end
    if(length(ind) >= 1)
        for i = 1: length(i)
            m = ind(i);
            for k=1:(N_lambda+1)
                Coll_X(k,1) = Traj{n}.X(k,1) - Traj{m}.X(k,1);
                Coll_X(k,2) = Traj{n}.X(k,2) - Traj{m}.X(k,2);
                Coll_X(k,3) = Traj{n}.X(k,3) - Traj{m}.X(k,3);
                for l=1:(N_lambda+1)
                    Coll_Y(l,1) = Traj{n}.X(l,1) - Traj{m}.X(l,1);
                    Coll_Y(l,2) = Traj{n}.X(l,2) - Traj{m}.X(l,2);
                    Coll_Y(l,3) = Traj{n}.X(l,3) - Traj{m}.X(l,3);
                    Coll_X_dot_Y = Coll_X(k,1)*Coll_Y(l,1) + Coll_X(k,2)*Coll_Y(l,2) + Coll_X(k,3)*Coll_Y(l,3);
                    constraintName = "Const" + num2str(total_constriants);
                    prob.Constraints.(constraintName) = Coll_X_dot_Y <= R(n,m)^2;
                    total_constriants = total_constriants + 1;
                end
            end
        end
    end
end

x0 = struct;
for n=1:N
   %x0 = setfield(x0,nodeName(n,:),in_drone(n).p0);
   
   for i=1:N_lambda+1
        setVal(i,:) =  drone(n).position{end};
   end
   x0.(nodeName(n,:)) = setVal;
end
options.Display = 'none';
lambda = solve(prob,x0,'Options',options);


for n=1:N
    results(n).Lambda = getfield(lambda,nodeName(n,:));
end


    function c = B(tau, T, k, K_h)
        c = factorial(T)/(factorial(tau)*factorial(T-tau))*(1 - k/K_h)^(T-tau)*(k/K_h)^(tau);
    end

    function c = B_prin(tau, T, k, K_h)
        if(tau==0)
            c=0;
        end
        c = T*(B(tau-1, T-1, k, K_h) - B(tau, T-1, k, K_h));
    end


end