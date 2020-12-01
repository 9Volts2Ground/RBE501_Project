function x = add_noise(x,P)

%Must define feedback_noise == true to add noise
if isfield(P, 'feedback_noise')
    
    %Only do noise if it's true
    if P.feedback_noise
        
        %Grab standard deviation of noise
        if isfield(P, 'noise_sigma')
            noise_sigma = P.noise_sigma;
        else
            noise_sigma = 0.02;
        end
        
        %Add offset value if not centered at 0
        if isfield(P, 'noise_median')
            noise_median = P.noise_median;
        else
            noise_median = 0;
        end 
        
        %Create noise vector with different noise values for each state
        state_noise = [normrnd( noise_median, noise_sigma ); ...    %x
                       normrnd( noise_median, noise_sigma ); ...    %y
                       normrnd( noise_median, noise_sigma ); ...    %z
                       normrnd( noise_median, noise_sigma ); ...    %xd
                       normrnd( noise_median, noise_sigma ); ...    %yd
                       normrnd( noise_median, noise_sigma ); ...    %zd
                       normrnd( noise_median, noise_sigma ); ...    %roll
                       normrnd( noise_median, noise_sigma ); ...    %pitch
                       normrnd( noise_median, noise_sigma ); ...    %yaw
                       normrnd( noise_median, noise_sigma ); ...    %rolld
                       normrnd( noise_median, noise_sigma ); ...    %pitchd
                       normrnd( noise_median, noise_sigma )];       %yawd
        
        %Compute new state vector with noise added in
        x(1:12) = x(1:12) + state_noise;
    end
    
end



end