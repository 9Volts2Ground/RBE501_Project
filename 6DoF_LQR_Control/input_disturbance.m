function u = input_disturbance(u, P)

%Must define input disturbance to add it
if isfield(P, 'input_disturbance')
    if P.input_disturbance
        
        if isfield(P, 'disturbance_offset')
            disturbance = P.disturbance_offset;
        else
            disturbance = [1 1 1 1]';
        end
        
        u(1:4) = u(1:4) + disturbance;
    end
end

end