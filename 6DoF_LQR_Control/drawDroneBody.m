function handle = drawDroneBody(x,y,z,roll,pitch,yaw,P,handle)

% %Test translation
% roll = 0;
% pitch = 0;
% yaw = 0;
% x = 0;
% y = 5;
% z = 0;

%Extract drone geometry from input P
w = P.w;
bh = P.base_height;
        
%Coordinates of all 8 corners of the cube
base_vertices = [0 0 0; 
               0 w 0; 
               w w 0; 
               w 0 0; 
               0 0 bh; 
               0 w bh; 
               w w bh; 
               w 0 bh]';

%Offset to bring the drone body centered at 0 (with z staying above 0)
center_offset = [-w/2 -w/2 0]';
for i=1:1:length(base_vertices(1,:))
    base_vertices(:,i) = base_vertices(:,i) + center_offset;
end

%Rotate base via roll-pitch-yaw angles
    %Note: We can update how we want to do orientation later
R_roll = rotrx(roll);
R_pitch = rotry(pitch);
R_yaw = rotrz(yaw);
R = R_roll*R_pitch*R_yaw;

rotated_vertices = R*base_vertices;

%Translate base via input x,y,z translation
translation = [x; y; z];

for i=1:1:length(rotated_vertices(1,:))
    translated_vertices(:,i) = rotated_vertices(:,i) + translation;
end

%Defines which indices make the 6 faces of the cube we are plotting
my_faces = [1 2 3 4; 
            2 6 7 3; 
            4 3 7 8; 
            1 5 8 4;
            1 2 6 5; 
            5 6 7 8];

% %Test plot the image
% figure()
% hold on
% grid on
% xlabel('x')
% ylabel('y')
% zlabel('z')
% axis equal
% patch('Vertices', translated_vertices', 'Faces', my_faces, 'FaceColor', 'b')

if isempty(handle)
    handle = patch('Vertices', translated_vertices', 'Faces', my_faces, 'FaceColor', 'b');
else
    set(handle,'Vertices', translated_vertices', 'Faces', my_faces);
end

end