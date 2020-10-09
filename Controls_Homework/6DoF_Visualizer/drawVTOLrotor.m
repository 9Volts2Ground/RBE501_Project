function handle = drawVTOLrotor(x,y,z,roll,pitch,yaw,quadrant,P,handle)

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


surf(prop_x_translated, prop_y_translated, prop_z_translated);

if isempty(handle)
    handle = surf(prop_x_translated, prop_y_translated, prop_z_translated);
else
    set(handle,'XData',Xprop_x_translated,'YData',prop_x_translated,'ZData',prop_z_translated);
end
    
%% Legacy code
% tt = 0:.1:2*pi;
% 
% d = P.d;
% if (side == 1)
%     d = d*-1;
% end
%            
% props_pts = [P.ew*cos(tt) +  d;
%              P.eh*sin(tt)];
%          
% % props_pts = [P.ew*cos(tt) + (zv + d);
% %              P.eh*sin(tt) + h];
%          
% % Tc = [(zv + d), (zv + d), (zv + d), (zv + d);
% %         h, h, h, h];
% % [(zv + d); h]*ones(2,4);
% 
% R = [cos(theta), -sin(theta);
%     sin(theta), cos(theta)];
% 
% % props_pts = props_pts + Tc;
% props_pts = R*props_pts;
% 
% for i=1:1:length(props_pts(1,:))
%    props_pts(:,i) = props_pts(:,i) + [zv; h]; 
% end
% 
% 
% X = props_pts(1,:);
% Y = props_pts(2,:);
% 
% if isempty(handle)
%     handle = fill(X,Y,'b');
% else
%     set(handle,'XData',X,'YData',Y);
% end

end