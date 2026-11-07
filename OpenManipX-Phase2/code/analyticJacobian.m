%% Analytic Jacobian Function

function J = analyticJacobian(q, params)
        c1 = cos(q(1));
        s1 = sin(q(1));
        c2 = cos(q(2));
        s2 = sin(q(2));
        c23 = cos(q(2) + q(3));
        s23 = sin(q(2) + q(3));
        c234 = cos(q(2) + q(3) + q(4));
        s234 = sin(q(2) + q(3) + q(4));

        A = params.a1*c2 + params.d2*s2 + params.a2*c23 + params.ell_e*c234;
        B = -params.a1*s2 + params.d2*c2 - params.a2*s23 - params.ell_e*s234;
        C = -params.a2*s23 - params.ell_e*s234;
        D = -params.ell_e*s234;
        E = -params.a2*c23 - params.ell_e*c234;
        F = -params.ell_e*c234;

        J = [-s1*A, c1*B, c1*C, c1*D;
              c1*A, s1*B, s1*C, s1*D;
              0,    -A,    E,    F];
end