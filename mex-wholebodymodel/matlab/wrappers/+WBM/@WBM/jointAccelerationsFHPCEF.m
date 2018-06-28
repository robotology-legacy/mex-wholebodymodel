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

function [ddq_j, fd_prms] = jointAccelerationsFHPCEF(obj, foot_conf, hand_conf, tau, fe_h, ac_h, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` with *foot*
    % and *hand pose corrections* and additionally *external forces* (*FHPCEF*)
    % that are acting on the hands of the humanoid robot -- *experimental*.
    %
    % The joint accelerations :math:`\ddot{q}_j` will be calculated as defined
    % in equation :eq:`joint_accelerations`. For a *closed-loop control system*
    % with *velocity* and *position regulation*, the calculated joint
    % acceleration vector depends on the given *contact constraints* and the
    % additional *external forces* that are acting on the hands of the
    % floating-base robot.
    %
    % The method can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsFHPCEF(foot_conf, hand_conf, tau, fe_h, ac_h[, ac_f], wf_R_b, wf_p_b, q_j, dq_j, v_b, nu)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and velocities:
    %
    %   - .. py:method:: jointAccelerationsFHPCEF(foot_conf, hand_conf, tau, fe_h, ac_h[, ac_f], Jc_f, djcdq_f, M, c_qv, dq_j, nu)
    %   - .. py:method:: jointAccelerationsFHPCEF(foot_conf, hand_conf, tau, fe_h, ac_h[, ac_f], dq_j, nu)
    %
    % Note:
    %   This method is still untested and should be considered as *experimental*.
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
    %   fe_h      (double, vector): :math:`(k \times 1)` vector of external forces
    %                               (in contact space :math:`\mathrm{C}_h`) that
    %                               are acting on the specified *contact points*
    %                               of the hands.
    %   ac_h      (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                               for the specified *hand contact points*.
    %
    %                               **Note:** If the hand accelerations are very small,
    %                               then the vector can also be *constant* or *zero*.
    %   ac_f      (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                               for the *foot contact points* (*optional*).
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
    % The given configuration structures specifying also the *desired poses*,
    % *angular velocities* and *control gains* for the position-regulation
    % system of the feet and hands.
    %
    % The variable :math:`k` indicates the *size* of the given *force* and
    % *acceleration vectors* in dependency of the specified feet and hands:
    %
    %   - :math:`k = 6`  -- only one foot/hand is defined.
    %   - :math:`k = 12` -- both feet/hands are defined.
    %
    % The given *external forces* and *foot accelerations* are either
    % *constant* or *zero*.
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
    %   [ddq_j[, fd_prms]]: 2-element tuple containing:
    %
    %      - **ddq_j** (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
    %        acceleration vector in :math:`[\si{\radian/{s^2}}]`.
    %      - **fd_prms**       (*struct*) -- Data structure for the parameter values
    %        of the forward dynamics calculation with the fields ``tau_gen``, ``f_c``,
    %        ``a_c`` and ``f_e`` (*optional*).
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerationsFHPCPL` and :meth:`WBM.contactForcesCLPCEF`.
    switch nargin
        case 13
            % with given foot contact acceleration:
            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};

                % compute the whole body dynamics, the Jacobians and the products
                % of the Jacobians derivative for the feet and hands ...
                [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = fullWholeBodyDynCS(obj, foot_conf, hand_conf, ...
                                                                             varargin{2:4}, dq_j, varargin{1,6});
            else
                % optimized mode:
                ac_f    = varargin{1,1};
                Jc_f    = varargin{1,2};
                djcdq_f = varargin{1,3};
                M       = varargin{1,4};
                c_qv    = varargin{1,5};
                dq_j    = varargin{1,6};

                [Jc_h, djcdq_h] = contactJacobiansCS(obj, hand_conf);
            end
            nu = varargin{1,7};
        case 11
            % with zero foot contact acceleration:
            if iscolumn(varargin{1,3})
                % normal mode:
                % wf_R_b = varargin{1}
                % wf_p_b = varargin{2}
                % q_j    = varargin{3}
                % v_b    = varargin{5}
                dq_j = varargin{1,4};

                [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = fullWholeBodyDynCS(obj, foot_conf, hand_conf, ...
                                                                             varargin{1:3}, dq_j, varargin{1,5});
            else
                % optimized mode:
                Jc_f    = varargin{1,1};
                djcdq_f = varargin{1,2};
                M       = varargin{1,3};
                c_qv    = varargin{1,4};
                dq_j    = varargin{1,5};

                [Jc_h, djcdq_h] = contactJacobiansCS(obj, hand_conf);
            end
            nu   = varargin{1,6};
            ac_f = zeroCtcAcc(obj, foot_conf);
        case 8 % optimized modes:
            % with foot contact acceleration:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = fullWholeBodyDynCS(obj, foot_conf, hand_conf);
        case 7
            % zero foot contact acceleration:
            dq_j = varargin{1,1};
            nu   = varargin{1,2};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = fullWholeBodyDynCS(obj, foot_conf, hand_conf);
        otherwise
            error('WBM::jointAccelerationsFHPCEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % compute the contact forces of the feet and hands
    % with friction and pose corrections (optimized mode):
    [fc_f, tau_gen] = footContactForcesPC(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu);
    [fc_h,~] = contactForcesCLPCEF(obj, hand_conf, tau, fe_h, ac_h, Jc_h, djcdq_h, M, c_qv, dq_j, nu);

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the contact constraints:
    J_c   = vertcat(Jc_f, Jc_h);
    f_c   = vertcat(fc_f, fc_h);
    Jc_t  = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fe_0 = zeroExtForces(obj, foot_conf);
        f_e = vertcat(fe_0, fe_h);
        a_c = vertcat(ac_f, ac_h);
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_e', f_e);
    end
end
