function tau_fr = wbm_frictionForces(dq_j, frict_c, frict_v)
    % WBM_FRICTIONFORCES computes the forces (torques) of friction F(dq_j) with a simple model.
    %
    % This function depends on the joint angle acceleration dq_j and on the friction coefficients of
    % the joints.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  none
    %
    %       Normal mode:
    %           dq_j    -- (nDoF x 1) joint angle acceleration vector (rad/s^2)
    %           frict_c -- (nDoF x 1) Coulomb friction coefficient vector of the joints
    %           frict_v -- (nDoF x 1) viscous friction coefficient vector of the joints
    %
    %   OUTPUT ARGUMENTS:
    %       tau_fr -- (nDoF x 1) friction force (torque) vector with negated values.
    %
    % Further details about the calculation are available in:
    %   [1] Modelling and Control of Robot Manipulators, L. Sciavicco & B. Siciliano, 2nd Edition, Springer, 2008,
    %       p. 133 & p. 141.
    %   [2] Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
    %       pp. 188-189, eq. (6.110)-(6.112).
    %   [3] Robotics, Vision & Control: Fundamental Algorithms in Matlab, Peter I. Corke, Springer, 2011,
    %       pp. 201-202, eq. (9.4) & (9.5).
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    epsilon = 1e-12; % min. value to treat a number as zero ...

    if (sum(dq_j ~= 0) <= epsilon) % if dq_j = 0
        tau_fr = zeros(size(dq_j,1),1);
        return
    end
    tau_cf = -frict_c .* sign(dq_j); % Coulomb friction torques
    tau_vf = -frict_v .* dq_j;       % viscous friction torques
    tau_fr =  tau_vf + tau_cf;       % friction torques
end
