function F=vtol_h_ctrl(in,P)
    z_c = in(1);
    z   = in(2);
    t   = in(3);
    
    % use a digital differentiator to find thetadot
    persistent zdot
    persistent z_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        zdot    = 0;
        z_d1    = 0;
    end
    
    zdot = P.beta*zdot + (1-P.beta)*((z-z_d1)/P.Ts);
    z_d1 = z;
    
    % construct the state
    x = [z; zdot];

    
    % compute the state feedback controller
    F_tilde = - P.Kh*x + P.Krh*z_c;
    % compute total torque
    F = sat( P.Fe + F_tilde, P.Force_max);
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end