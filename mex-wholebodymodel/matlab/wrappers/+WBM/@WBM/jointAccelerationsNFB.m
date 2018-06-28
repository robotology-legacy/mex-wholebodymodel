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

function [ddq_j, fd_prms] = jointAccelerationsNFB(obj, tau, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` of a
    % humanoid robot without a *floating base* (*NFB*) or by exclusion of it.
    %
    % Since the given robot is defined without a floating base, there are no
    % contact forces :math:`f_c` to the ground, i.e. :math:`f_c = 0`. The
    % behavior of the model is the same as a robot with a fixed base.
    %
    % The joint accelerations will be calculated as defined in equation
    % :eq:`joint_accelerations` and in :cite:`Lilly1992`, but without the
    % contact forces :math:`f_c`, such that:
    %
    %   .. math::
    %       :label: joint_acceleration_no_ctc_forces
    %
    %       \ddot{q}_j = M^{\text{-}1}\cdot (\tau_{gen} - C(q_j, \dot{q}_j))
    %
    % The method can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   - .. py:method:: jointAccelerationsNFB(tau, wf_R_b, wf_p_b, q_j, dq_j, v_b)
    %   - .. py:method:: jointAccelerationsNFB(tau, wf_R_b, wf_p_b, q_j, nu)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and the joint velocities:
    %
    %   - .. py:method:: jointAccelerationsNFB(tau, M, c_qv[, dq_j])
    %   - .. py:method:: jointAccelerationsNFB(tau[, dq_j])
    %
    % Arguments:
    %   tau    (double, vector): :math:`(n \times 1)` torque force vector
    %                            for the joints and the base of the robot
    %                            with :math:`n = n_{dof} + 6`.
    %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
    %                            (orientation) from the base frame *b*
    %                            to world frame *wf* (*optional*).
    %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
    %                            the base frame *b* to the world frame *wf*
    %                            (*optional*).
    %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
    %                            vector in :math:`[\si{\radian}]` (*optional*).
    %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                            vector in :math:`[\si{\radian/s}]` (*optional*).
    %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                            vector (*optional*).
    %   nu     (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
    %                            velocity vector (generalized base velocity and
    %                            joint velocity) -- *optional*.
    % Other Parameters:
    %   M    (double, matrix): :math:`(n \times n)` generalized mass matrix
    %                          of the robot (*optional*).
    %   c_qv (double, vector): :math:`(n \times 1)` generalized bias force
    %                          vector of the robot (*optional*).
    % Returns:
    %   [ddq_j[, fd_prms]]: 2-element tuple containing:
    %
    %      - **ddq_j** (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
    %        acceleration vector in :math:`[\si{\radian/{s^2}}]`.
    %      - **fd_prms**       (*struct*) -- Data structure for the parameter values
    %        of the forward dynamics calculation with the field ``tau_gen`` (*optional*).
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerationsNFBEF` and :meth:`WBM.jointAccelerationsNFBPL`.
    %
    % References:
    %   :cite:`Lilly1992`, p. 82, eq. (5.2).

    % References:
    %   [Lil92] Lilly, Kathryn: Efficient Dynamic Simulation of Robotic Mechanisms.
    %           Springer, 1992, p. 82, eq. (5.2).
    switch nargin
        case 7 % normal modes:
            % generalized forces with friction:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
            tau_gen = tau + tau_fr;              % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                 % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix.
            % compute the whole body dynamics ...
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv] = wholeBodyDyn(obj, wf_R_b_arr, varargin{1,2}, varargin{1,3}, dq_j, varargin{1,5});
        case 6
            % without friction:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            nu = varargin{1,4};

            len  = obj.mwbm_model.ndof + 6;
            dq_j = nu(7:len,1);
            v_b  = nu(1:6,1);

            tau_gen = tau;

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv] = wholeBodyDyn(obj, wf_R_b_arr, varargin{1,2}, varargin{1,3}, dq_j, v_b);

        case 5 % optimized modes:
            % with friction:
            % dq_j = varargin{3}
            M    = varargin{1,1};
            c_qv = varargin{1,2};

            tau_fr  = frictionForces(obj, varargin{1,3});
            tau_gen = tau + tau_fr;
        case 4
            % without friction:
            M    = varargin{1,1};
            c_qv = varargin{1,2};

            tau_gen = tau;
        case 3
            % with friction:
            % dq_j = varargin{1}

            tau_fr  = frictionForces(obj, varargin{1,1});
            tau_gen = tau + tau_fr;

            [M, c_qv] = wholeBodyDyn(obj);
        case 2
            % without friction:
            tau_gen   = tau;
            [M, c_qv] = wholeBodyDyn(obj);
        otherwise
            error('WBM::jointAccelerationsNFB: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % remove the floating base part ...
    M    = M(7:end,7:end);
    c_qv = c_qv(7:end,1);

    % Joint Acceleration q_ddot without floating base (derived from the dyn. equations of motion):
    % Note: Since the robot has no floating base, there are no contact forces f_c to the
    %       ground, i.e. f_c = 0. The behavior is the same as a robot with a fixed base.
    %
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    ddq_j = M \ (tau_gen - c_qv);
    ddq_j = vertcat(zeros(6,1), ddq_j);

    if (nargout == 2)
        % set the forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen);
    end
end
