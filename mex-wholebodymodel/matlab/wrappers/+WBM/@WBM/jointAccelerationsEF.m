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

function [ddq_j, fd_prms] = jointAccelerationsEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` with
    % additional *external forces* (*EF*) that are acting on the specified
    % contact links of the given floating-base robot -- *experimental*.
    %
    % The joint accelerations :math:`\ddot{q}_j` will be calculated as defined
    % in equation :eq:`joint_accelerations`. In order to obtain the joint
    % accelerations from the robot system, the method computes at first the
    % *contact forces* :math:`f_c` of the feet and of the contact links, in
    % dependency of the *external forces* that are acting on the contact links
    % of the robot.
    %
    % The method assumes that *only the feet* of the robot may have contact with
    % the *ground* and can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by the
    % base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsEF(foot_conf, clnk_conf, tau, fe_c, ac[, ac_f], wf_R_b, wf_p_b, q_j, dq_j, v_b)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given joint velocities:
    %
    %   .. py:method:: jointAccelerationsEF(foot_conf, clnk_conf, tau, fe_c, ac[, ac_f], dq_j)
    %
    % Note:
    %   This method is still untested and should be considered as *experimental*.
    %
    % Arguments:
    %   foot_conf      (struct): Configuration structure to specify the *qualitative
    %                            state* of the feet.
    %
    %                            The data structure specifies which foot is currently
    %                            in contact with the ground.
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
    %   ac_f   (double, vector): :math:`(k \times 1)` mixed acceleration vector for
    %                            the specified *foot contact points* (*optional*).
    %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix (orientation)
    %                            from the base frame *b* to world frame *wf*
    %                            (*optional*).
    %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from the base
    %                            frame *b* to the world frame *wf* (*optional*).
    %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
    %                            vector in :math:`[\si{\radian}]` (*optional*).
    %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                            vector in :math:`[\si{\radian/s}]`.
    %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                            vector (*optional*).
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
    %   :meth:`WBM.jointAccelerations`, :meth:`WBM.jointAccelerationsPL` and
    %   :meth:`WBM.contactForcesEF`.
    switch nargin
        case 12 % normal modes:
            % with foot contact acceleration:
            % wf_R_b = varargin{2}
            % wf_p_b = varargin{3}
            % q_j    = varargin{4}
            % v_b    = varargin{6}
            ac_f = varargin{1,1};
            dq_j = varargin{1,5};

            % compute the whole body dynamics of all contact constraints
            % of the given contact link configurations ...
            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                     varargin{2:4}, dq_j, varargin{1,6});
        case 11
            % with zero foot contact acceleration:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                     varargin{1:3}, dq_j, varargin{1,5});
        case 8 % optimized modes:
            % with foot contact acceleration:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};

            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        case 7
            % with zero foot contact acceleration:
            dq_j = varargin{1,1};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        otherwise
            error('WBM::jointAccelerationsEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % compute the contact forces with friction (optimized mode):
    [fc_f, tau_gen] = footContactForces(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j);
    [fc,~] = contactForcesEF(obj, tau, fe_c, ac, Jc, djcdq, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the contact constraints:
    J_c   = vertcat(Jc_f, Jc);
    f_c   = vertcat(fc_f, fc);
    Jc_t  = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (tau - c_qv - Jc.'*(-f_c))

    if (nargout == 2)
        % set the forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'a_c', ac, 'f_e', fe_c);
    end
end
