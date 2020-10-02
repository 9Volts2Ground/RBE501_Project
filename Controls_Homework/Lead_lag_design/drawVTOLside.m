function handle = drawVTOLside(zv,h,theta,zt,P,side,handle)

tt = 0:.1:2*pi;

d = P.d;
if (side == 1)
    d = d*-1;
end
           
props_pts = [P.ew*cos(tt) +  d;
             P.eh*sin(tt)];
         
% props_pts = [P.ew*cos(tt) + (zv + d);
%              P.eh*sin(tt) + h];
         
% Tc = [(zv + d), (zv + d), (zv + d), (zv + d);
%         h, h, h, h];
% [(zv + d); h]*ones(2,4);

R = [cos(theta), -sin(theta);
    sin(theta), cos(theta)];

% props_pts = props_pts + Tc;
props_pts = R*props_pts;

for i=1:1:length(props_pts(1,:))
   props_pts(:,i) = props_pts(:,i) + [zv; h]; 
end


X = props_pts(1,:);
Y = props_pts(2,:);

if isempty(handle)
    handle = fill(X,Y,'b');
else
    set(handle,'XData',X,'YData',Y);
end

end