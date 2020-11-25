function handle = drawTarget(zv,h,theta,zt,P,handle)

L = P.wt;

pts = [-L/2 L/2 L/2 -L/2;
        -L/2 -L/2 L/2 L/2];
zoffset = zt*[1 1 1 1; 0 0 0 0];

pts = pts + zoffset;
X = pts(1,:);
Y = pts(2,:);

    if isempty(handle)
        handle = fill(X,Y,'b');
    else
        set(handle,'XData',X,'YData',Y);
    end

end