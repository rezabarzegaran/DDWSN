function probability = probabDist(x,y)
% This is the event PDF.
K=3;
%U= 4*rand(2,1)-[2;2];
U = [1.2; 1.4];
S = 0.5*eye(2);
X = [x;y];
D = U-X;
probability = (K/(2*pi*sqrt(det(S))))*exp(-0.5*transpose(D)*(S^-1)*(D));
end