

function Fr = FT2fr(F,T)

run('vtol_param.m')

Fr = .5*F - T/(2*P.d);
% Fl = .5*F - T/(2*P.d);
end