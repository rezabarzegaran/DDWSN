function combinedobject = makeDroneView(hg, angle)
% This function generates drone view cone graphics
D2R = pi/180;
height = 1;  % Height of the cone
alpha_angle = angle*D2R;
radius = height * tan(alpha_angle);   % Radius of the base
num_points = 50;  % Number of points for the circular base

% Create a set of points for the circular base
theta = linspace(0, 2*pi, num_points);
x_base = radius * cos(theta);
y_base = radius * sin(theta);
z_base = zeros(1, num_points)-height;

% Create the vertices for the cone's lateral surface
x_lateral = [x_base, 0];  % Add a point at the apex to close the surface
y_lateral = [y_base, 0];  % Add a point at the apex to close the surface
z_lateral = [zeros(1, num_points)-height, 0];  % Set the z-coordinates for the lateral surface
% Plot the base of the cone
cone(1)=plot3(x_base, y_base, z_base, 'g');

% Plot the lateral surface of the cone
cone(2)=patch(x_lateral, y_lateral, z_lateral, 'r');
alpha(cone(2),0.2);
combinedobject = hgtransform('parent',hg );
set(cone,'parent',combinedobject)
end