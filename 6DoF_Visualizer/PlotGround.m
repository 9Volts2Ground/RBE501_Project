function PlotGround(x,y,handle)

ground_size = 10;

x_limits = [x-ground_size, x+ground_size];
y_limits = [y-ground_size, y+ground_size];

[x_ground,y_ground] = meshgrid(x_limits(1):x_limits(2), y_limits(1):y_limits(2));

z_ground = zeros(size(x_ground, 1));

% color = [0,0,0];

[m,n] = size(x_ground);
color = zeros(m, n, 3);

surf(x_ground, y_ground, z_ground, color);



end