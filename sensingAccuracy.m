function accuracy = sensingAccuracy(omegat,p,sParam)
% This function calculates the sensing accuracy
omega = [omegat(1) omegat(2) 0];
v = omega - p;
distance = norm(v);
if(distance < 3)
    distance;
end
angp = 0;
if(v(1)~=0)
    angp = atand(v(2)/v(1))-sParam(4);
end
angt = sParam(7)- acosd(v(3)/distance);
%angt = angt + sParam(7)*-sign(angt);
if(angp >= 45)
    angp = 90-angp;
elseif (angp <= -45)
    angp = -90-angp;
end
%angp = angp + 180*-sign(angp);
mud = 1-(1/(1+exp(-sParam(1)*(distance-sParam(2)))));
mup= 1/(1+exp(-sParam(3)*(angp+sParam(5)))) - 1/(1+exp(-sParam(3)*(angp-sParam(5))));
mut= 1/(1+exp(-sParam(6)*(angt+sParam(8)))) - 1/(1+exp(-sParam(6)*(angt-sParam(8))));
mup = 1;

accuracy = mud*mup*mut;


end