function out = vtol_h_ctrl(in,P)
    z_c = in(1);
    z   = in(2);
    t   = in(3);
    
    % use a digital differentiator to find thetadot
    persistent xhat
%     persistent z_d1
    persistent F
    
    % reset persistent variables at start of simulation
    if t<P.Ts
        z_d1    = 0;
        F = 0;
        xhat = [0 0]';
    end
    
    % Implements the observer
    N = 10;
    for i=1:1:N
        xhat = xhat + (P.Ts/N)*(P.Ah*xhat + P.Bh*(F - P.Fe) + P.Lh*(z - P.Ch*xhat));
    end
    
    %Unpack the vector
    zhat = xhat(1);
    zdothat = xhat(2);
    
%     zdothat = P.beta*zdothat + (1-P.beta)*((zhat-z_d1)/P.Ts);
%     z_d1 = zhat;
%     
%     % construct the state
%     x = [zhat; zdothat];

    
    % compute the state feedback controller
    F_tilde = - P.Kh*xhat + P.Krh*z_c;
    % compute total torque
    F = sat( P.Fe + F_tilde, P.Force_max);
    out = [F; zhat];
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end