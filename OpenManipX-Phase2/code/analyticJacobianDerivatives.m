function [dJ1, dJ2, dJ3, dJ4] = analyticJacobianDerivatives(q, params)


    c1 = cos(q(1));            
    s1 = sin(q(1));
    c2 = cos(q(2));            
    s2 = sin(q(2));
    c23 = cos(q(2)+q(3));       
    s23 = sin(q(2)+q(3));
    c234 = cos(q(2)+q(3)+q(4));  
    s234 = sin(q(2)+q(3)+q(4));

    % Same intermediate terms as analyticJacobian.m
    A =  params.a1*c2 + params.d2*s2 + params.a2*c23 + params.ell_e*c234;   % radial reach
    B = -params.a1*s2 + params.d2*c2 - params.a2*s23 - params.ell_e*s234;   % dA/dq2
    C = -params.a2*s23 - params.ell_e*s234;                                 % dA/dq3
    D = -params.ell_e*s234;                                                 % dA/dq4
    E = -params.a2*c23 - params.ell_e*c234;                                 % dC/dq2 = dC/dq3
    F = -params.ell_e*c234;                                                 % dD/dq_i

    dJ1 = [-c1*A, -s1*B, -s1*C, -s1*D;
           -s1*A,  c1*B,  c1*C,  c1*D;
            0,     0,     0,     0   ];

    dJ2 = [-s1*B, -c1*A,  c1*E,  c1*F;
            c1*B, -s1*A,  s1*E,  s1*F;
            0,    -B,    -C,    -D   ];

    dJ3 = [-s1*C,  c1*E,  c1*E,  c1*F;
            c1*C,  s1*E,  s1*E,  s1*F;
            0,    -C,    -C,    -D   ];

    dJ4 = [-s1*D,  c1*F,  c1*F,  c1*F;
            c1*D,  s1*F,  s1*F,  s1*F;
            0,    -D,    -D,    -D   ];
end
