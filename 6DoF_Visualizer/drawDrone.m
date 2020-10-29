function drawDrone(u,P)

%Unpack input from Simulink
x = u(1);
y = u(2);
z = u(3);
roll = u(4);
pitch = u(5);
yaw = u(6);
t = u(7);

%Plotting handles
persistent VTOLbody_handle;
persistent VTOLrotor1_handle;
persistent VTOLrotor2_handle;
persistent VTOLrotor3_handle;
persistent VTOLrotor4_handle;
persistent Ground_handle;

%Plotting limits
persistent x_max
persistent x_min
persistent y_max
persistent y_min
persistent z_max
plot_buffer = 1.5;

%First time plotting
if t==0
    
    x_max = x + plot_buffer;
    x_min = x - plot_buffer;
    y_max = y + plot_buffer;
    y_min = y - plot_buffer;
    z_max = z + plot_buffer;
    
    fprintf('Initializing plot \n')
    figure()
    clf
    axis equal
    hold on
    grid on
    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')
    view(30, 10)  %Force angled camera view
    %Figure out how to draw a plane to represent the ground
    VTOLbody_handle = drawDroneBody(x,y,z,roll,pitch,yaw,P,[]);
    VTOLrotor1_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,1,P,[]);
    VTOLrotor2_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,2,P,[]);
    VTOLrotor3_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,3,P,[]);
    VTOLrotor4_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,4,P,[]);
    Ground_handle = drawGround(x,y,[]);
    xlim([x_min, x_max])
    ylim([y_min, y_max])
    zlim([0, z_max])
    
%Update plots every other call
else    
    
    fprintf('calculate plot_limits... \n')
    %Update max plotting limits
    x_max = max([x_max, x + plot_buffer]);
    x_min = min([x_min, x - plot_buffer]);
    y_max = max([y_max, y + plot_buffer]);
    y_min = min([y_min, y - plot_buffer]);
    z_max = max([z_max, z + plot_buffer]);
    
    fprintf('draw body... \n')
    drawDroneBody(x,y,z,roll,pitch,yaw,P,VTOLbody_handle);    
    fprintf('draw r1... \n')
    drawDroneRotor(x,y,z,roll,pitch,yaw,1,P,VTOLrotor1_handle);
    fprintf('draw r2... \n')
    drawDroneRotor(x,y,z,roll,pitch,yaw,2,P,VTOLrotor2_handle);
    fprintf('draw r3... \n')
    drawDroneRotor(x,y,z,roll,pitch,yaw,3,P,VTOLrotor3_handle);
    fprintf('draw r4... \n')
    drawDroneRotor(x,y,z,roll,pitch,yaw,4,P,VTOLrotor4_handle);
    fprintf('draw ground... \n')
    drawGround(x,y,Ground_handle);
    
    fprintf('update lims... \n')
    xlim([x_min, x_max])
    ylim([y_min, y_max])
    zlim([0, z_max])
end

end