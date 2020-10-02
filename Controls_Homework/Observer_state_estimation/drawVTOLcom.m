function handle = drawVTOLcom(zv,h,theta,zt,P,handle)

w = P.w;

center_pts = [-w/2, w/2, w/2, -w/2;
               -w/2, -w/2, w/2, w/2];
         
Tc = [zv, zv, zv, zv;
      h, h, h, h];
% [zv, h]*ones(2,4);

R = [cos(theta), -sin(theta);
    sin(theta), cos(theta)];

center_pts = R*center_pts;
center_pts = center_pts + Tc;

X = center_pts(1,:);
Y = center_pts(2,:);

if isempty(handle)
    handle = fill(X,Y,'b');
else
    set(handle,'XData',X,'YData',Y);
end

end