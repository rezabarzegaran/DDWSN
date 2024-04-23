function edges = calcEdges(p0,R)
%This functions calculates all edges between drones.
% There is an edge between two drones if their distance is less than their
% communication range R_i,j
p = transpose(p0);
D = dist(p);
edges = D <= R;
end