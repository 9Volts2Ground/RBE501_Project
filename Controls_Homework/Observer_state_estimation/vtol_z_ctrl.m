function out = vtol_z_ctrl(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    % use a digital differentiator to find zdot and thetadot
%     persistent z_d1
%     persistent theta_d1
    persistent xhat
    persistent T
    
    % reset persistent variables at start of simulation
    if t<P.Ts
        z_d1        = 0;
        theta_d1    = 0;
        T = 0;
        xhat = [0 0 0 0]';
    end

    %Implement observer
    N = 10;
    for i=1:1:N
        x_input = [z, theta]';
       xhat = xhat + (P.Ts/N)*(P.Az*xhat + P.Bz*T + P.Lz*(x_input - P.Cz*xhat)); 
    end
    
    %Unpack xhat
    zhat = xhat(1);
    thetahat = xhat(2);
    zdothat = xhat(3);
    thetadothat = xhat(4);
    
%     zdothat = P.beta*zdothat + 2/(2*P.sigma + P.Ts)*(zhat - z_d1);
%     thetadothat = P.beta*thetadothat + 2/(2*P.sigma + P.Ts)*(thetahat - theta_d1);
% 
%     z_d1 = zhat;
%     theta_d1 = thetahat;

%     % construct the state
%     x = [zhat; thetahat; zdothat; thetadothat];

    % compute the state feedback controller
    
    T = sat( -P.Kz*xhat + P.Krz*z_r, P.Torque_max);
    
    out = [T; zhat; thetahat];
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else,                   out = in;
    end
end