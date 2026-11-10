% control law for manipulability maximization

function [qdot,m] = controlLawTask2(q, x, x_des, params, Kp, k0)
    % Compute the error
    err = x_des - x;
    % Compute the Jacobian
    J = analyticJacobian(q, params);
    Jp = pinv(J);
    % Compute the manipulability measure
    m = prod(svd(J));
    % Compute the derivative of the Jacobian
    [dJ1, dJ2, dJ3, dJ4] = analyticJacobianDerivatives(q, params);
    
     dm = m * [trace(Jp*dJ1); trace(Jp*dJ2); trace(Jp*dJ3); trace(Jp*dJ4)];
    
    % Null-space projector and control law
    N = eye(4)-Jp*J;
    qdot = Jp*Kp*err + N*(k0*dm);
end