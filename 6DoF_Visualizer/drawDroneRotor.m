function handle = drawDroneRotor(x,y,z,roll,pitch,yaw,quadrant,P,handle)

% %Test translation
% roll = 0;
% pitch = 0;
% yaw = 0;
% x = 0;
% y = 0;
% z = 0;
% quadrant = 4;

radius = P.prop_length/2;
[prop_x, prop_y, prop_z] = cylinder(radius);

prop_z = prop_z*P.prop_thickness + P.base_height; %Scale how tall we plot the cylinder. Make it sit above the drone base

%Translate rotor to corner of the drone
if quadrant == 1
    prop_x = prop_x + P.w/2;
    prop_y = prop_y - P.w/2;
elseif quadrant == 2
    prop_x = prop_x + P.w/2;
    prop_y = prop_y + P.w/2;
elseif quadrant == 3
    prop_x = prop_x - P.w/2;
    prop_y = prop_y + P.w/2;
elseif quadrant == 4
    prop_x = prop_x - P.w/2;
    prop_y = prop_y - P.w/2;
else
    fprintf('drawVTOLrotor: quadrant out of bounds. quadrant = %d (range is 1:4) \n', quadrant);
    exit
end

prop_bottom = [prop_x(1,:); prop_y(1,:); prop_z(1,:)];
prop_top    = [prop_x(2,:); prop_y(2,:); prop_z(2,:)];

%Tilt rotor via roll-pitch-yaw angles
    %Note: We can update how we want to do orientation later
R_roll = rotrx(roll);
R_pitch = rotry(pitch);
R_yaw = rotrz(yaw);
R = R_roll*R_pitch*R_yaw;
    
prop_bottom = R*prop_bottom;
prop_top = R*prop_top;

%Translate the prop
translation = [x; y; z];
prop_bottom_translated = prop_bottom + translation;
prop_top_translated = prop_top + translation;

prop_x_translated = [prop_bottom_translated(1,:); prop_top_translated(1,:)];
prop_y_translated = [prop_bottom_translated(2,:); prop_top_translated(2,:)];
prop_z_translated = [prop_bottom_translated(3,:); prop_top_translated(3,:)];


% surf(prop_x_translated, prop_y_translated, prop_z_translated);

if isempty(handle)
    handle = surf(prop_x_translated, prop_y_translated, prop_z_translated);
else
    set(handle,'XData',prop_x_translated,'YData',prop_y_translated,'ZData',prop_z_translated);
end
    

end