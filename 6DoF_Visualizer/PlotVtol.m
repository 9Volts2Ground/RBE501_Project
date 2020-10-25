function PlotVtol(u,P)

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
    figure()
    clf
    axis equal
    hold on
    grid on
    %Figure out how to draw a plane to represent the ground
    VTOLbody_handle = drawVTOLbody(x,y,z,roll,pitch,yaw,P,[]);
    VTOLrotor1_handle = drawVTOLrotor(x,y,z,roll,pitch,yaw,1,P,[]);
    VTOLrotor2_handle = drawVTOLrotor(x,y,z,roll,pitch,yaw,2,P,[]);
    VTOLrotor3_handle = drawVTOLrotor(x,y,z,roll,pitch,yaw,3,P,[]);
    VTOLrotor4_handle = drawVTOLrotor(x,y,z,roll,pitch,yaw,4,P,[]);
%Update plots every other call
else
    drawVTOLbody(x,y,z,roll,pitch,yaw,P,VTOLbody_handle);
    drawVTOLrotor(x,y,z,roll,pitch,yaw,1,P,VTOLrotor1_handle);
    drawVTOLrotor(x,y,z,roll,pitch,yaw,2,P,VTOLrotor2_handle);
    drawVTOLrotor(x,y,z,roll,pitch,yaw,3,P,VTOLrotor3_handle);
    drawVTOLrotor(x,y,z,roll,pitch,yaw,4,P,VTOLrotor4_handle);
end

%% Legacy code
% h = u(1);   %VTOL, vertical
% zv = u(2);  %VTOL, horizontal
% theta = u(3); %VTOL, angle
% zt = u(4);  %Target, horizontal
% t = u(5);
% 
% persistent VTOLcom_handle;
% persistent VTOLr_handle;
% persistent VTOLl_handle;
% persistent Target_handle;
% 
%     if t==0
%         figure(), clf
%         axis equal
%         hold on
%         grid on
%         plot([-5,5],[0,0],'k--');   %Draw ground line for visual reference
%         axis([-5,5,-3,6]);
%         VTOLcom_handle = drawVTOLcom(zv,h,theta,zt,P,[]);
%         VTOLr_handle = drawVTOLside(zv,h,theta,zt,P,0,[]);
%         VTOLl_handle = drawVTOLside(zv,h,theta,zt,P,1,[]);
%         Target_handle = drawTarget(zv,h,theta,zt,P,[]);
%       
%     else
%         drawVTOLcom(zv,h,theta,zt,P,VTOLcom_handle);
%         drawVTOLside(zv,h,theta,zt,P,0,VTOLr_handle);
%         drawVTOLside(zv,h,theta,zt,P,1,VTOLl_handle);
%         drawTarget(zv,h,theta,zt,P,Target_handle);
% 
%     end

end