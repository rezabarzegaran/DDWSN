function [gilbert, dist] = CreateGilbert(n,drone,R)
%This function calculates the Gilbert Graph on the lesser neighborhood of a
%drone considering the communication range R.
    gilbert = [];
    dist = [];
    counter = 1;
    for i = (n-1):-1:1
        distance = norm(drone(n).position{end}-drone(i).position{end});
        if(distance<=R(n,i))
            gilbert(counter) = i;
            dist(counter) = distance;
            counter = counter +1;
        end
    end

end