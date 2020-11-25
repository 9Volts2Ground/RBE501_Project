function u = ground_limit(u)

z = u(3);

%Prevent the drone from falling below the ground limit
if z < 0
    z = 0;
end

fprintf('We crashed!!! Altitude was: %d \n', u(3))
u(3) = z;

end