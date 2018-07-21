% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January-May, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU-project CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
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

function tau_j = inverseDynamics(obj, varargin)
    % Computes the inverse dynamics of a rigid-body system. It depends on the
    % *state parameter triplet* :math:`[q_j, \dot{q}_j, v_b]`, the *joint angle
    % accelerations* :math:`\ddot{q}_j` and the *generalized base accelerations*
    % :math:`\dot{v}_b`.
    %
    % If the *generalized floating-base acceleration* :math:`\dot{v}_b` is
    % unknown, then the inverse dynamics problem changes to a *hybrid-dynamics
    % problem* (:cite:`Featherstone2008`, p. 183). In general the equation of
    % motion is given as follows:
    %
    %   .. math::
    %      :label: equation_of_motion
    %
    %      \tau_j = M \ddot{q}_j + C(q_j,\dot{q}_j) + \tau_{fr}
    %
    % where :math:`C(q_j,\dot{q}_j)` denotes the generalized bias force.
    %
    % In contrast to the fixed-based system, the *equation of motion* for a
    % *floating-base system* is defined as follows:
    %
    %   .. math::
    %      :label: equation_of_motion_fb
    %
    %      \begin{bmatrix} 0\\ \tau_j \end{bmatrix} =
    %      \begin{bmatrix}
    %         M_{00}   & M_{01}\\
    %         M_{01}^T & M_{11}
    %      \end{bmatrix}
    %      \cdot
    %      \begin{bmatrix} \dot{v}_b\\ \ddot{q}_j \end{bmatrix} +
    %      \begin{bmatrix} h_0\\ h_1 \end{bmatrix} +
    %      \begin{bmatrix} 0\\ \tau_{fr} \end{bmatrix}
    %
    % The matrix on the left side describes the :math:`(n \times n)` *mass matrix*
    % :math:`\mathbf{M}` of the robot and the vector :math:`h = [h_0, h_1]^T` is
    % a :math:`(n \times 1)` vector representing the *generalized bias forces*
    % (Coriolis, centrifugal and gravity forces), where :math:`n = n_{dof}+6`.
    % The inverse dynamics model will be solved by using the first row of the
    % above equation :eq:`equation_of_motion_fb` to obtain the *generalized base
    % acceleration*:
    %
    %   .. math::
    %      :label: base_velocity
    %
    %      \dot{v}_b = \text{-}M_{00}^{\text{-}1} (M_{01} \ddot{q}_j + h_0)
    %
    % The method can be called by one of the given modes:
    %
    % **Normal mode** -- Computes the inverse dynamics of the robot, specified
    % by the base orientation, the positions, the velocities and accelerations:
    %
    %   .. py:method:: inverseDynamics(wf_R_b, wf_p_b, q_j, dq_j, v_b, ddq_j[, dv_b])
    %
    % **Optimized mode** -- Computes the inverse dynamics of the robot at the
    % current state of the robot system:
    %
    %   .. py:method:: inverseDynamics(dq_j, ddq_j[, dv_b])
    %
    % Arguments:
    %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix (orientation)
    %                            from the base frame *b* to world frame *wf*.
    %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from the base
    %                            frame *b* to the world frame *wf*.
    %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions vector
    %                            in :math:`[\si{\radian}]`.
    %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                            vector in :math:`[\si{\radian/s}]`.
    %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                            vector (Cartesian and rotational velocity of
    %                            the base).
    %   ddq_j  (double, vector): :math:`(n_{dof} \times 1)` joint angle acceleration
    %                            vector in :math:`[\si{\radian/{s^2}}]`.
    %   dv_b   (double, vector): :math:`(6 \times 1)` generalized base acceleration
    %                            vector (*optional*).
    % Returns:
    %   tau_j (double, vector): :math:`(n \times 1)` generalized force vector of
    %   the joints and the base of the robot with :math:`n = n_{dof} + 6`.
    %
    % References:
    %   - :cite:`Featherstone2008`, chapter 9.3-9.5, pp. 180-184, eq. (9.13) and (9.24).
    %   - :cite:`Khalil2011`, p. 14, eq. (36) and (37).
    %   - :cite:`Shah2012`, p. 119, eq. (7.1) and (7.2).

    % References:
    %   [Fea08] Featherstone, Roy: Rigid Body Dynamics Algorithms. Springer, 2008,
    %           Chapter 9.3-9.5, pp. 180-184, eq. (9.13) and (9.24).
    %   [Kha11] Khalil, Wisama: Dynamic Modeling of Robots Using Newton-Euler Formulation.
    %           In: Informatics in Control, Automation and Robotics. Lecture Notes in Electrical Engineering,
    %           Volume 89, Springer, 2011, p. 14, eq. (36) and (37).
    %   [SSD12] Shah, S. V.; Saha, S. K.; Dutt, J. K.: Dynamics of Tree-Type Robotic Systems.
    %           In: Intelligent Systems, Control and Automation: Science and Engineering, Volume 62,
    %           Springer, 2012, p. 119, eq. (7.1) and (7.2).

    % wf_R_b = varargin{1}
    % wf_p_b = varargin{2}
    % q_j    = varargin{3}
    % dq_j   = varargin{4}
    % v_b    = varargin{5}
    % ddq_j  = varargin{6}
    % dv_b   = varargin{7}
    switch nargin
        case 8 % normal modes:
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            tau    = mexWholeBodyModel('inverse-dynamics', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                       varargin{1,4}, varargin{1,5}, varargin{1,6},  varargin{1,7});
            tau_fr = frictionForces(obj, varargin{1,4}); % friction torques (negative values)
            tau_fr = vertcat(zeros(6,1), tau_fr);

            tau_j = tau + tau_fr;
            return
        case 7
            % if dv_b, the acceleration (linear & angular acceleration)
            % of the robot base is not given or unknown ...
            wf_p_b = varargin{1,2};
            q_j    = varargin{1,3};
            dq_j   = varargin{1,4};
            ddq_j  = varargin{1,6};

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
            c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, varargin{1,5});
        case 4 % optimized modes:
            % dq_j  = varargin{1}
            % ddq_j = varargin{2}
            % dv_b  = varargin{3}

            % Note: The same vector dq_j is already stored inside of the mex-subroutine. Because
            %       before any function can be used in optimized mode, the function "setState"
            %       must be called previously to update the state parameters q_j, dq_j and v_b.
            tau    = mexWholeBodyModel('inverse-dynamics', varargin{1,2}, varargin{1,3});
            tau_fr = frictionForces(obj, varargin{1,1});
            tau_fr = vertcat(zeros(6,1), tau_fr);

            tau_j = tau + tau_fr;
            return
        case 3
            % if dv_b is not given or unknown ...
            dq_j  = varargin{1,1};
            ddq_j = varargin{1,2};

            M    = mexWholeBodyModel('mass-matrix');
            c_qv = mexWholeBodyModel('generalized-forces');
    otherwise
        error('WBMBase::inverseDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end

    tau_fr = frictionForces(obj, dq_j);
    tau_fr = vertcat(zeros(6,1), tau_fr);

    %% Generalized floating-base acceleration for a hybrid-dynamic system:
    %
    %  Equation of motion for a floating-base system:
    %
    %       |   0   |   | M_00      M_01 |   | dv_b  |   | h_0 |   |   0    |
    %       |       | = |                | * |       | + |     | + |        |
    %       | tau_j |   | M_01^T    M_11 |   | ddq_j |   | h_1 |   | tau_fr |
    %
    %  where M is the (n+6)-by-(n+6) mass matrix, h = (h_0, h_1)^T a
    %  (n+6)-by-1 vector representing the generalized bias forces and
    %  dv_b = -(M_00)^(-1) * (M_01 * ddq_j + h_0) denotes the base acceleration.
    dv_b  = generalizedBaseAcc(obj, M, c_qv, ddq_j);
    ddq_j = vertcat(dv_b, ddq_j); % joint accelerations

    tau_j = M*ddq_j + c_qv + tau_fr;
end
