function p = NewPredictDroneDynamics(p0,v0,u, dt,current)
% This function is used for drone prediction dynamics

cx=0;
%ax=100*dt;
ax=1;
cy=0;
%ay=100*dt;
ay=1;
cz=0;
%az=100*dt;
az=1;

A = [ 1 0 0 dt 0 0;
      0 1 0 0 dt 0;
      0 0 1 0 0 dt;
      -ax 0 0 1 0 0;
      0 -ay 0 0 1 0;
      0 0 -az 0 0 1];
B = [0 0 0;
     0 0 0;
     0 0 0;
     ax 0 0;
     0 ay 0;
     0 0 az];

q0 = transpose([p0 v0]);

q = A^current*q0;

for i = 1: current

    q = q + A^(current - i)*B*transpose(u(i,:));
end

p = [q(1) q(2) q(3)];
%v = [q(4) q(5) q(6)];
end