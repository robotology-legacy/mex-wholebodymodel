function tau_fr = wbm_frictionForces(dq_j, frict_c, frict_v)
    % WBM_FRICTIONFORCES computes the frictional torque-forces F(dq_j) by using a simplified model.
    %
    % The function depends on the current joint angle acceleration dq_j and on the given friction
    % coefficients of the joints.
    %
    % Input Arguments:
    %   dq_j    -- (n_dof x 1) joint angle acceleration vector in [rad/s^2].
    %   frict_c -- (n_dof x 1) Coulomb friction coefficient vector of the joints.
    %   frict_v -- (n_dof x 1) viscous friction coefficient vector of the joints.
    %
    % Output Arguments:
    %   tau_fr -- (n_dof x 1) frictional torque-force vector with negated values.
    %
    % Further details about the calculation are available in:
    %   [1] Siciliano, B.; Sciavicco, L.; Villani, L.; Oriolo, G.:
    %       Modelling and Control of Robot Manipulators.
    %       2nd Edition, Springer, 2008, p. 133 & p. 141.
    %   [2] Craig, John J.: Introduction to Robotics: Mechanics and Control.
    %       3rd Edition, Pearson/Prentice Hall, 2005, pp. 188-189, eq. (6.110)-(6.112).
    %   [3] Corke, Peter I.: Robotics, Vision & Control: Fundamental Algorithms in Matlab.
    %       Springer, 2011, pp. 201-202, eq. (9.4) & (9.5).

    % Copyright (C) 2017 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    %
    % This function is part of the Whole-Body Model Library for Matlab (WBML).
    %
    % Permission is granted to copy, distribute, and/or modify the WBM-Library
    % under the terms of the GNU Lesser General Public License, Version 2.1
    % or any later version published by the Free Software Foundation.
    %
    % The WBM-Library is distributed in the hope that it will be useful,
    % but WITHOUT ANY WARRANTY; without even the implied warranty of
    % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    % GNU Lesser General Public License for more details.
    %
    % A copy of the GNU Lesser General Public License can be found along
    % with the WBML. If not, see <http://www.gnu.org/licenses/>.

    epsilon = 1e-12; % min. value to treat a number as zero ...

    if (sum(dq_j ~= 0) <= epsilon) % if dq_j = 0
        tau_fr = zeros(size(dq_j,1),1);
        return
    end
    tau_cf = -frict_c .* sign(dq_j); % Coulomb friction torques
    tau_vf = -frict_v .* dq_j;       % viscous friction torques
    tau_fr =  tau_vf + tau_cf;       % friction torques
end
