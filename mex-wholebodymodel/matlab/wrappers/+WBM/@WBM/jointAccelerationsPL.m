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

function [ddq_j, fd_prms] = jointAccelerationsPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` of a humanoid
    % robot with *payload forces* (*PL*) that are acting on the hands of the robot.
    %
    % Since the given robot is defined without a floating base, there are no
    % contact forces :math:`f_c` to the ground, i.e. :math:`f_c = 0`. The
    % behavior of the model is the same as a robot with a fixed base.
    %
    % The joint accelerations :math:`\ddot{q}_j` will be calculated as defined
    % in equation :eq:`joint_accelerations`. In order to obtain the joint
    % accelerations from the robot system, the method computes at first the
    % *contact forces* :math:`f_c` of the feet and of the contact links, in
    % dependency of the *external forces* that are acting on the contact links
    % of the robot.
    %
    % The method assumes that *only the feet* of the robot may have contact
    % with the *ground* and can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsPL(foot_conf, hand_conf, tau, fhTotCWrench, f_cp[, ac_f], wf_R_b, wf_p_b, q_j, dq_j, v_b)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given joint velocities:
    %
    %   .. py:method:: jointAccelerationsPL(foot_conf, hand_conf, tau, fhTotCWrench, f_cp[, ac_f], dq_j)
    %
    % Arguments:
    %   foot_conf             (struct): Configuration structure to specify the *qualitative
    %                                   state* of the feet.
    %
    %                                   The data structure specifies which foot is currently
    %                                   in contact with the ground.
    %   hand_conf             (struct): Configuration structure to specify the *qualitative
    %                                   state* of the hands.
    %
    %                                   The data structure specifies which hand is currently
    %                                   in contact with the payload object.
    %   tau           (double, vector): :math:`(n \times 1)` torque force vector for the
    %                                   joints and the base of the robot with
    %                                   :math:`n = n_{dof} + 6`.
    %   fhTotCWrench (function_handle): Function handle to a specific *total
    %                                   contact wrench function* in *contact space*
    %                                   :math:`\mathrm{C_h = \{C_1,\ldots,C_n\}}`
    %                                   of the hands that will be applied by the
    %                                   robot model.
    %   f_cp          (double, vector): Force vector or scalar applied to a grasped
    %                                   object at the *contact points*
    %                                   :math:`{}^{\small O}p_{\small C_i}` from the
    %                                   contact frames :math:`\mathrm{\{C_1,\ldots,C_n\}}`
    %                                   of the hands to the origin frame :math:`\mathrm{O}`
    %                                   at the CoM of the object.
    %
    %                                   The vector length :math:`l` of the applied
    %                                   forces depends on the chosen *contact model*
    %                                   and if only one hand or both hands are involved
    %                                   in grasping an object, such that :math:`l = h\cdot s`
    %                                   with size :math:`s \in \{1,3,4\}` and the number
    %                                   of hands :math:`h \in \{1,2\}`.
    %
    %                                   **Note:** The z-axis of a contact frame
    %                                   :math:`\mathrm{C_{i \in \{1,\ldots,n\}}}`
    %                                   points in the direction of the inward surface
    %                                   normal at the point of contact
    %                                   :math:`{}^{\small O}p_{\small C_i}`. If the
    %                                   chosen contact model is *frictionless*, then
    %                                   each applied force to the object is a scalar,
    %                                   otherwise a vector.
    %   ac_f          (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                                   for the specified *foot contact points* with
    %                                   the size of :math:`k = 6` (one foot) or
    %                                   :math:`k = 12` (both feet) -- *optional*.
    %
    %                                   **Note:** The given *foot accelerations* are either
    %                                   *constant* or *zero*.
    %   wf_R_b        (double, matrix): :math:`(3 \times 3)` rotation matrix (orientation)
    %                                   from the base frame *b* to world frame *wf*
    %                                   (*optional*).
    %   wf_p_b        (double, vector): :math:`(3 \times 1)` position vector from the base
    %                                   frame *b* to the world frame *wf* (*optional*).
    %   q_j           (double, vector): :math:`(n_{dof} \times 1)` joint positions
    %                                   vector in :math:`[\si{\radian}]` (*optional*).
    %   dq_j          (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                                   vector in :math:`[\si{\radian/s}]`.
    %   v_b           (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                                   vector (*optional*).
    % Returns:
    %   [ddq_j[, fd_prms]]: 2-element tuple containing:
    %
    %      - **ddq_j** (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
    %        acceleration vector in :math:`[\si{\radian/{s^2}}]`.
    %      - **fd_prms**       (*struct*) -- Data structure for the parameter values
    %        of the forward dynamics calculation with the fields ``tau_gen``, ``f_c``,
    %        ``a_c`` and ``f_pl`` (*optional*).
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerations`, :meth:`WBM.jointAccelerationsEF`, :meth:`WBM.handAccelerations`,
    %   :meth:`WBM.handPayloadForces` and :meth:`WBM.contactForcesEF`.
    switch nargin
        case 12 % normal modes:
            % wf_R_b = varargin{2}
            % wf_p_b = varargin{3}
            % q_j    = varargin{4}
            % v_b    = varargin{6}
            ac_f = varargin{1,1};
            dq_j = varargin{1,5};

            % get the mixed accelerations of the hands ...
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, ...
                                                   varargin{2:4}, dq_j, varargin{1,6});
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        case 11
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, ...
                                                   varargin{1:3}, dq_j, varargin{1,5});
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        case 8 % optimized modes:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};

            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin{1:2});
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        case 7
            dq_j = varargin{1,1};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, dq_j); % with friction
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        otherwise
            error('WBM::jointAccelerationsPL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % mixed velocity of the hands at the contact links {lnk} ...
    wf_v_lnk = a_prms.Jc_h * dq_j;

    % apply velocity and acceleration saturation:
    % (to prevent overload & integration problems)
    wf_v_lnk = WBM.utilities.mbd.satVel(wf_v_lnk);
    wf_a_lnk = WBM.utilities.mbd.satAcc(wf_a_lnk);

    % calculate the payload forces of the hands in contact space {c} = {lnk}:
    f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk);

    % compute the contact forces of the hands (with friction):
    [fc_h,~] = contactForcesEF(obj, tau, f_pl, wf_a_lnk, a_prms.Jc_h, a_prms.djcdq_h, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j in dependency of the
    % contact forces of the feet and the current payload forces of the hands:
    J_c = vertcat(Jc_f, a_prms.Jc_h);
    f_c = vertcat(a_prms.fc_f, fc_h);
    % f_c   = vertcat(a_prms.fc_f, f_pl);
    Jc_t  = J_c.';
    ddq_j = M \ (a_prms.tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        a_c = vertcat(ac_f, wf_a_lnk);
        fd_prms = struct('tau_gen', a_prms.tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_pl', f_pl);
    end
end
