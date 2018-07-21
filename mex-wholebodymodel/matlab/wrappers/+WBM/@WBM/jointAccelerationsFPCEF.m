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

function [ddq_j, fd_prms] = jointAccelerationsFPCEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` with *foot
    % pose corrections* and and additionally *external forces* (*FPCEF*) that
    % are acting on the specified contact links of the floating-base robot --
    % *experimental*.
    %
    % The equation for solving the joint accelerations :math:`\ddot{q}_j` of
    % a closed chain (see eq. :eq:`joint_accelerations`) is derived from the
    % dynamic equations of motion. In order to obtain the joint accelerations
    % from the *closed-loop control system* with *position-regulation*
    % (velocity and position correction) of the feet, the method computes at
    % first the *contact forces* :math:`f_c` of the feet and of the contact
    % links, in dependency of the *external forces* that are acting on the
    % contact links of the robot.
    %
    % The method assumes that *only the feet* of the robot may have contact with
    % the *ground* and can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsFPCEF(foot_conf, clnk_conf, tau, fe_c, ac[, ac_f], wf_R_b, wf_p_b, q_j, dq_j, v_b, nu)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and velocities:
    %
    %   - .. py:method:: jointAccelerationsFPCEF(foot_conf, clnk_conf, tau, fe_c, ac[, ac_f], Jc_f, djcdq_f, M, c_qv, dq_j, nu)
    %   - .. py:method:: jointAccelerationsFPCEF(foot_conf, clnk_conf, tau, fe_c, ac[, ac_f], nu)
    %
    % Note:
    %   This method is still untested and should be considered as *experimental*.
    %
    % Arguments:
    %   foot_conf      (struct): Configuration structure to specify the *qualitative
    %                            state* of the feet.
    %
    %                            The data structure specifies which foot is currently
    %                            in contact with the ground. It specifies also the
    %                            *desired poses*, *angular velocities* and *control
    %                            gains* for the position-regulation system of the feet.
    %   clnk_conf      (struct): Configuration structure to specify the *qualitative
    %                            state* of at most two *contact links*.
    %
    %                            The data structure specifies which link is currently
    %                            in contact with the ground or an object.
    %   tau    (double, vector): :math:`(n \times 1)` torque force vector for the
    %                            joints and the base of the robot with
    %                            :math:`n = n_{dof} + 6`.
    %   fe_c   (double, vector): :math:`(k \times 1)` vector of external forces
    %                            (in contact space :math:`\mathrm{C}`) that are
    %                            acting on the specified contact links.
    %   ac     (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                            for the *contact points* of the contact links.
    %
    %                            **Note:** If the accelerations are very small,
    %                            then the vector can also be *constant* or *zero*.
    %   ac_f   (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                            for the specified *foot contact points*
    %                            (*optional*).
    %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix (orientation)
    %                            from the base frame *b* to world frame *wf*
    %                            (*optional*).
    %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from the base
    %                            frame *b* to the world frame *wf* (*optional*).
    %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
    %                            vector in :math:`[\si{\radian}]` (*optional*).
    %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                            vector in :math:`[\si{\radian/s}]` (*optional*).
    %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                            vector (*optional*).
    %   nu     (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
    %                            velocity vector (generalized base velocity and
    %                            joint velocity).
    %
    % The variable :math:`k` indicates the *size* of the given *force* and
    % *acceleration vectors* in dependency of the specified contact links
    % and feet:
    %
    %   - :math:`k = 6`  -- only one link/foot is defined.
    %   - :math:`k = 12` -- both links/feet are defined.
    %
    % The given *external forces* and *foot accelerations* are either
    % *constant* or *zero*.
    %
    % Other Parameters:
    %   Jc    (double, matrix): :math:`(6 m \times n)` Jacobian of the
    %                           *contact constraints* in contact space
    %                           :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`
    %                           (*optional*).
    %   djcdq (double, vector): :math:`(6 m \times 1)` product vector of
    %                           the time derivative of the contact Jacobian
    %                           :math:`\dot{J}_c` and the joint velocity
    %                           :math:`\dot{q}_j` (*optional*).
    %   M     (double, matrix): :math:`(n \times n)` generalized mass matrix
    %                           of the robot (*optional*).
    %   c_qv  (double, vector): :math:`(n \times 1)` generalized bias force
    %                           vector of the robot (*optional*).
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
    %   :meth:`WBM.jointAccelerationsFPC`, :meth:`WBM.jointAccelerationsFPCPL` and
    %   :meth:`WBM.contactForcesEF`.
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

                % compute the whole body dynamics and for each contact constraint the
                % Jacobian and the corresponding product of the Jacobian derivative ...
                [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                         varargin{2:4}, dq_j, varargin{1,6});
            else
                % optimized mode:
                ac_f    = varargin{1,1};
                Jc_f    = varargin{1,2};
                djcdq_f = varargin{1,3};
                M       = varargin{1,4};
                c_qv    = varargin{1,5};
                dq_j    = varargin{1,6};

                [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf);
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

                [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                         varargin{1:3}, dq_j, varargin{1,5});
            else
                % optimized mode:
                Jc_f    = varargin{1,1};
                djcdq_f = varargin{1,2};
                M       = varargin{1,3};
                c_qv    = varargin{1,4};
                dq_j    = varargin{1,5};

                [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf);
            end
            nu   = varargin{1,6};
            ac_f = zeroCtcAcc(obj, foot_conf);
        case 8 % optimized modes:
            % with foot contact acceleration:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        case 7
            % zero foot contact acceleration:
            dq_j = varargin{1,1};
            nu   = varargin{1,2};
            ac_f = zeroCtcAcc(obj, foot_conf);

            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        otherwise
            error('WBM::jointAccelerationsFPCEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % compute the contact forces with friction (optimized mode):
    [fc_f, tau_gen] = footContactForcesPC(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu); % with PC
    [fc,~] = contactForcesEF(obj, tau, fe_c, ac, Jc, djcdq, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the contact constraints:
    J_c   = vertcat(Jc_f, Jc);
    f_c   = vertcat(fc_f, fc);
    Jc_t  = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fe_0 = zeroExtForces(obj, foot_conf);
        f_e = vertcat(fe_0, fe_c);
        a_c = vertcat(ac_f, ac);
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_e', f_e);
    end
end
