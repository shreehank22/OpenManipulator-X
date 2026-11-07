%% control law for position control

function [qdot, m] = controlLawTask1(q, x, x_des, params, Kp)
    %% Compute the error
    err = x_des - x;
    %% Compute the Jacobian
    J = analyticJacobian(q, params);
    %% Compute the control law
    qdot = pinv(J) * (Kp * err);
    %% Compute the manipulability measure
    m = prod(svd(J));
end