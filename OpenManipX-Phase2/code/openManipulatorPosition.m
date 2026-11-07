%% Local functions (MATLAB requires these at the end of a script)
function x = openManipulatorPosition(q, params)
    x = [params.a0 + cos(q(1))*(params.a1*cos(q(2)) + params.d2*sin(q(2)) + params.a2*cos(q(2) + q(3)) + params.ell_e*(cos(q(2) + q(3) + q(4))));
    sin(q(1))*(params.a1*cos(q(2)) + params.d2*sin(q(2)) + params.a2*cos(q(2) + q(3)) + params.ell_e*(cos(q(2) + q(3) + q(4))));
    params.d1 - params.a1*sin(q(2)) + params.d2*cos(q(2)) - params.a2*sin(q(2) + q(3)) - params.ell_e*(sin(q(2) + q(3) + q(4)))];
end