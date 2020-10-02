function PlotVtol(u,P)

h = u(1);   %VTOL, vertical
zv = u(2);  %VTOL, horizontal
theta = u(3); %VTOL, angle
zt = u(4);  %Target, horizontal
t = u(end);

persistent VTOLcom_handle;
persistent VTOLr_handle;
persistent VTOLl_handle;
persistent Target_handle;

    if t==0
        figure(), clf
        axis equal
        hold on
        grid on
        plot([-2,7],[0,0],'k--');
        axis([-2,7,-2,4]);
        VTOLcom_handle = drawVTOLcom(zv,h,theta,zt,P,[]);
        VTOLr_handle = drawVTOLside(zv,h,theta,zt,P,0,[]);
        VTOLl_handle = drawVTOLside(zv,h,theta,zt,P,1,[]);
        Target_handle = drawTarget(zv,h,theta,zt,P,[]);
      
    else
        drawVTOLcom(zv,h,theta,zt,P,VTOLcom_handle);
        drawVTOLside(zv,h,theta,zt,P,0,VTOLr_handle);
        drawVTOLside(zv,h,theta,zt,P,1,VTOLl_handle);
        drawTarget(zv,h,theta,zt,P,Target_handle);

    end

end