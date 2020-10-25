function drawDrone(u,P)

%Unpack input from Simulink
x = u(1);
y = u(2);
z = u(3);
roll = u(4);
pitch = u(5);
yaw = u(6);
t = u(7);

persistent VTOLbody_handle;
persistent VTOLrotor1_handle;
persistent VTOLrotor2_handle;
persistent VTOLrotor3_handle;
persistent VTOLrotor4_handle;

%First time plotting
if t==0
    
    fprintf('Initializing plot \n')
    figure()
    clf
    axis equal
    hold on
    grid on
    %Figure out how to draw a plane to represent the ground
    VTOLbody_handle = drawDroneBody(x,y,z,roll,pitch,yaw,P,[]);
    VTOLrotor1_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,1,P,[]);
    VTOLrotor2_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,2,P,[]);
    VTOLrotor3_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,3,P,[]);
    VTOLrotor4_handle = drawDroneRotor(x,y,z,roll,pitch,yaw,4,P,[]);
    drawGround(x,y);
%Update plots every other call
else    
    drawDroneBody(x,y,z,roll,pitch,yaw,P,VTOLbody_handle);    
    drawDroneRotor(x,y,z,roll,pitch,yaw,1,P,VTOLrotor1_handle);
    drawDroneRotor(x,y,z,roll,pitch,yaw,2,P,VTOLrotor2_handle);
    drawDroneRotor(x,y,z,roll,pitch,yaw,3,P,VTOLrotor3_handle);
    drawDroneRotor(x,y,z,roll,pitch,yaw,4,P,VTOLrotor4_handle);
end

end