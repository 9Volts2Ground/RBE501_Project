function handle = drawGround(x,y,handle)

% ground_size = 2;

% x_limits = [x-ground_size, x+ground_size];
% y_limits = [y-ground_size, y+ground_size];

persistent ground_x_max
persistent ground_x_min
persistent ground_y_max
persistent ground_y_min

if isempty(handle)
    ground_x_max = x + 3;
    ground_x_min = x - 3;
    ground_y_max = y + 3;
    ground_y_min = y - 3;
else
    ground_x_max = max([ground_x_max, x+3]);
    ground_x_min = min([ground_x_min, x-3]);
    ground_y_max = max([ground_y_max, y+3]);
    ground_y_min = min([ground_y_min, y-3]);
end

x_limits = [ground_x_min, ground_x_max];
y_limits = [ground_y_min, ground_y_max];

[x_ground,y_ground] = meshgrid(x_limits(1):x_limits(2), y_limits(1):y_limits(2));

z_ground = zeros(size(x_ground, 1));

% color = [0,0,0];

[m,n] = size(x_ground);
color = zeros(m, n, 3);

surf(x_ground, y_ground, z_ground, color);

if isempty(handle)
    handle = surf(x_ground, y_ground, z_ground, color);
else
    set(handle,'XData', x_ground, 'YData', y_ground, 'ZData', z_ground);
end

end