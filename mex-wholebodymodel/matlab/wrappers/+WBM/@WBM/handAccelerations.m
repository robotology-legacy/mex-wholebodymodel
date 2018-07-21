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

function [ac_h, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin)
    % Calculates the *mixed hand accelerations* (end-effector accelerations)
    % :math:`a_{c_h}` of a humanoid robot in contact (operational) space
    % :math:`\mathrm{C}_h`.
    %
    % Using equation :eq:`joint_accelerations` for the joint accelerations
    % :math:`\ddot{q}_j`, the formula to calculate the *mixed accelerations*
    % :math:`a_{c_h}` at the specified *contact links* of the hands
    % (end-effectors) can be expressed as follows:
    %
    %   .. math::
    %      :label: mixed_hand_accelerations
    %
    %      a_{c_h} = \ddot{x}_h = J_{c_h}M^{\text{-}1} (\tau_{gen} - C(q_j, \dot{q}_j) -
    %      (J^{T}_{c_f}\cdot \text{-}f_{c_f})) + \dot{J}_{c_h}\dot{q}_j\:,
    %
    % where the matrix :math:`J_{c_h}` denotes the contact Jacobian of the hands
    % and :math:`J^{T}_{c_f}\cdot \text{-}f_{c_f}` represents the contact forces
    % of the feet in joint space.
    %
    % The method can be called in two different modes:
    %
    % **Normal mode** -- Computes the mixed accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: handAccelerations(foot_conf, hand_conf, tau, ac_f, wf_R_b, wf_p_b, q_j, dq_j, v_b[, nu])
    %
    % **Optimized mode** -- Computes the mixed accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and velocities:
    %
    %   - .. py:method:: handAccelerations(foot_conf, hand_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu)
    %   - .. py:method:: handAccelerations(foot_conf, hand_conf, tau, ac_f, dq_j[, nu])
    %
    % Arguments:
    %   foot_conf         (struct): Configuration structure to specify the
    %                               *qualitative state* of the feet.
    %
    %                               The data structure specifies which foot is
    %                               currently in contact with the ground.
    %   hand_conf         (struct): Configuration structure to specify the
    %                               *qualitative state* of the hands.
    %
    %                               The data structure specifies which hand is
    %                               currently in contact with the ground, or an
    %                               object, or a wall.
    %   tau       (double, vector): :math:`(n \times 1)` torque force vector for
    %                               the joints and the base of the robot with
    %                               :math:`n = n_{dof} + 6`.
    %   ac_f      (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                               for the specified *foot contact points* with the
    %                               size of :math:`k = 6` (one foot) or :math:`k = 12`
    %                               (both feet) -- *optional*.
    %
    %                               **Note:** The given *foot accelerations* are either
    %                               *constant* or *zero*.
    %   wf_R_b    (double, matrix): :math:`(3 \times 3)` rotation matrix (orientation)
    %                               from the base frame *b* to world frame *wf*
    %                               (*optional*).
    %   wf_p_b    (double, vector): :math:`(3 \times 1)` position vector from the base
    %                               frame *b* to the world frame *wf* (*optional*).
    %   q_j       (double, vector): :math:`(n_{dof} \times 1)` joint positions
    %                               vector in :math:`[\si{\radian}]` (*optional*).
    %   dq_j      (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                               vector in :math:`[\si{\radian/s}]`.
    %   v_b       (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                               vector (*optional*).
    %   nu        (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
    %                               velocity vector (generalized base velocity and
    %                               joint velocity).
    %
    % In dependency of the situation, the given configuration structures can
    % also specify the *desired poses*, *angular velocities* and *control
    % gains* for the position-regulation system of the feet and hands.
    %
    % Other Parameters:
    %   Jc_f    (double, matrix): :math:`(6 m \times n)` Jacobian of the
    %                             *foot contact constraints* in contact space
    %                             :math:`\mathrm{C}_f` (*optional*).
    %   djcdq_f (double, vector): :math:`(6 m \times 1)` product vector of the
    %                             time derivative of the contact Jacobian
    %                             :math:`\dot{J}^{f}_c` for the feet and the
    %                             joint velocity :math:`\dot{q}_j` (*optional*).
    %   M       (double, matrix): :math:`(n \times n)` generalized mass matrix
    %                             of the robot (*optional*).
    %   c_qv    (double, vector): :math:`(n \times 1)` generalized bias force
    %                             vector of the robot (*optional*).
    %
    % The variable :math:`m` denotes the *number of contact constraints* of
    % the robot and :math:`n = n_{dof} + 6`.
    %
    % Returns:
    %   [ac_h[, a_prms]]: 2-element tuple containing:
    %
    %      - **ac_h** (*double, vector*) -- :math:`(6 m \times 1)` mixed acceleration
    %        vector of the hands in :math:`[\si{\radian/{s^2}}]` with either
    %        :math:`m = 1` or :math:`m = 2`.
    %      - **a_prms**       (*struct*) -- Data structure of the calculated hand
    %        accelerations parameters with the fields:
    %
    %           a) ``M``, ``c_qv``, ``Jc_f``, ``Jc_h``, ``djcdq_f``, ``djcdq_h``, ``fc_f``, ``tau_gen`` or
    %           b) ``Jc_h``, ``djcdq_h``, ``fc_f``, ``tau_gen``
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerations` and :meth:`WBM.handVelocities`.
    %
    % References:
    %   :cite:`Lilly1992`, p. 82, eq. (5.9).

    % References:
    %   [Lil92] Lilly, Kathryn: Efficient Dynamic Simulation of Robotic Mechanisms.
    %           Springer, 1992, p. 82, eq. (5.9).

    % check the contact state (CS) of the hands ...
    hand_idx = getContactIdx(obj, hand_conf);
    if ~hand_idx
        % no contacts:
        ac_h = obj.ZERO_CVEC_12;
        if (nargout == 2)
            a_prms = struct(); % empty structure ...
        end
        return
    end

    f_data = true;
    switch nargin
        case 11 % with pose corrections:
            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};
                nu   = varargin{1,7}; % mixed generalized velocity

                [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, foot_conf, hand_idx, ...
                                                                           varargin{2:4}, dq_j, varargin{1,6});
            else
                % optimized mode:
                ac_f    = varargin{1,1};
                Jc_f    = varargin{1,2};
                djcdq_f = varargin{1,3};
                M       = varargin{1,4};
                c_qv    = varargin{1,5};
                dq_j    = varargin{1,6};
                nu      = varargin{1,7};

                [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx);
                f_data = false;
            end
            % compute the foot contact forces with pose corrections (PC) and the
            % corresponding generalized forces with friction ...
            [fc_f, tau_gen] = footContactForcesPC(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu);
        case 10
            % normal mode (without pose corrections):
            % wf_R_b = varargin{2}
            % wf_p_b = varargin{3}
            % q_j    = varargin{4}
            % v_b    = varargin{6}
            ac_f = varargin{1,1};
            dq_j = varargin{1,5};

            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, foot_conf, hand_idx, ...
                                                                       varargin{2:4}, dq_j, varargin{1,6});
            % compute the foot contact forces with the corresponding generalized forces ...
            [fc_f, tau_gen] = footContactForces(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j); % with friction
        case 7 % optimized modes:
            % with pose corrections:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, foot_conf, hand_idx);
            [fc_f, tau_gen] = footContactForcesPC(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu);
        case 6
            % without pose corrections:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};

            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, foot_conf, hand_idx);
            [fc_f, tau_gen] = footContactForces(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j);
        otherwise
            error('WBM::handAccelerations: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % calculate the joint accelerations with the foot contact constraints ...
    Jcf_t = Jc_f.';
    ddqj_f = M \ (tau_gen + Jcf_t*fc_f - c_qv);

    % calculate the mixed accelerations of the hands at the contact links:
    ac_h = Jc_h*ddqj_f + djcdq_h;

    if (nargout == 2)
        % data structure of the calculated acceleration parameters:
        if f_data
            % with foot data Jc_f, djcdq_f, M and c_qv ...
            a_prms = struct('M', M, 'c_qv', c_qv, 'Jc_f', Jc_f, 'Jc_h', Jc_h, 'djcdq_f', djcdq_f, ...
                            'djcdq_h', djcdq_h, 'fc_f', fc_f, 'tau_gen', tau_gen);
            return
        end
        % else, without Jc_f, djcdq_f, M and c_qv (existing outside) ...
        a_prms = struct('Jc_h', Jc_h, 'djcdq_h', djcdq_h, 'fc_f', fc_f, 'tau_gen', tau_gen);
    end
end
%% END of handAccelerations.


%% WHOLE BODY DYNAMICS FOR FEET & HANDS:

function [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, foot_conf, hand_idx, wf_R_b, wf_p_b, q_j, dq_j, v_b) % FH ... Foot and Hand, CS ... Contact State
    switch nargin
        case 8
            % normal mode:
            wf_R_b_arr = reshape(wf_R_b, 9, 1);
            [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
            [Jc_h, djcdq_h] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, hand_idx);
        case 3
            % optimized mode:
            [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf);
            [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx);
        otherwise
            error('wholeBodyDynFHCS: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
