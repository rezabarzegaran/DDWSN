function [p,v] = DisDroneDynamics(p0,v0,u, v_bound, a_bound, dt)
% This is the Discrete drone dynamics function.
cx=0;
ax=1;
cy=0;
ay=1;
cz=0;
az=1;



if(ax*(u(1)-p0(1)) > a_bound(2))
    cx=a_bound(2)*dt;
    ax=0;
elseif(ax*(u(1)-p0(1)) < a_bound(1))
    cx=a_bound(1)*dt;
    ax=0;
end

if(ay*(u(2)-p0(2)) > a_bound(2))
    cy=a_bound(2)*dt;
    ay=0;
elseif(ay*(u(2)-p0(2)) < a_bound(1))
    cy=a_bound(1)*dt;
    ay=0;
end
if(az*(u(3)-p0(3)) > a_bound(2))
    cz=a_bound(2)*dt;
    az=0;
elseif(az*(u(3)-p0(3)) < a_bound(1))
    cz=a_bound(1)*dt;
    az=0;
end



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

C = [0;
     0;
     0;
     cx;
     cy;
     cz];

q0 = transpose([p0 v0]);

q = A*q0+B*transpose(u)+C;

if(q(4) > v_bound(2))
    q(4) = v_bound(2);
elseif(q(4) < v_bound(1))
    q(4) = v_bound(1);
end

if(q(5) > v_bound(2))
    q(5) = v_bound(2);
elseif(q(5) < v_bound(1))
    q(5) = v_bound(1);
end

if(q(6) > v_bound(2))
    q(6) = v_bound(2);
elseif(q(6) < v_bound(1))
    q(6) = v_bound(1);
end


p = [q(1) q(2) q(3)];
v = [q(4) q(5) q(6)];
end
