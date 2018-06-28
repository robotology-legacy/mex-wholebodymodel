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

function [ddq_j, fd_prms] = jointAccelerationsFHPCPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` with *foot*
    % and *hand pose corrections* and additionally *payload forces* (*FHPCPL*)
    % that are acting on the hands of the humanoid robot -- *experimental*.
    %
    % The joint accelerations :math:`\ddot{q}_j` will be calculated as defined
    % in equation :eq:`joint_accelerations`. For a *closed-loop control system*
    % with *velocity* and *position regulation*, the calculated joint
    % acceleration vector depends on the given *contact constraints* and the
    % additional *payload forces* that are acting on the hands of the
    % floating-base robot.
    %
    % The method assumes that *only the feet* of the robot have contact with
    % the *ground* and can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsFHPCPL(foot_conf, hand_conf, tau, fhTotCWrench, f_cp[, ac_f], wf_R_b, wf_p_b, q_j, dq_j, v_b, nu)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and velocities:
    %
    %   - .. py:method:: jointAccelerationsFHPCPL(foot_conf, hand_conf, tau, fhTotCWrench, f_cp[, ac_f], Jc_f, djcdq_f, M, c_qv, dq_j, nu)
    %   - .. py:method:: jointAccelerationsFHPCPL(foot_conf, hand_conf, tau, fhTotCWrench, f_cp[, ac_f], dq_j, v_b, nu)
    %
    % Note:
    %   This method is still untested and should be considered as *experimental*.
    %
    % Arguments:
    %   foot_conf             (struct): Configuration structure to specify the
    %                                   *qualitative state* of the feet.
    %
    %                                   The data structure specifies which foot
    %                                   is currently in contact with the ground.
    %   hand_conf             (struct): Configuration structure to specify the
    %                                   *qualitative state* of the hands.
    %
    %                                   The data structure specifies which hand
    %                                   is currently in contact with the payload
    %                                   object.
    %   tau           (double, vector): :math:`(n \times 1)` torque force vector
    %                                   for the joints and the base of the robot
    %                                   with :math:`n = n_{dof} + 6`.
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
    %                                   **Note:** The given *foot accelerations* are
    %                                   either *constant* or *zero*.
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
    %   nu            (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
    %                                   velocity vector (generalized base velocity and
    %                                   joint velocity).
    %
    % The given configuration structures specifying also the *desired poses*,
    % *angular velocities* and *control gains* for the position-regulation
    % system of the feet and hands.
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
    %        ``a_c`` and ``f_pl`` (*optional*).
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerationsFHPCEF`, :meth:`WBM.contactForcesCLPCEF`,
    %   :meth:`WBM.handAccelerations` and :meth:`WBM.handPayloadForces`.
    switch nargin
        case 13
            % get the mixed accelerations of the hands ...
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin{1:7});

            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};

                [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
            else
                % optimized mode:
                % djcdq_f = varargin{3}
                ac_f = varargin{1,1};
                Jc_f = varargin{1,2};
                M    = varargin{1,4};
                c_qv = varargin{1,5};
                dq_j = varargin{1,6};
            end
            nu = varargin{1,7};
        case 12
            ac_f = zeroCtcAcc(obj, foot_conf);
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, varargin{1:6});

            if iscolumn(varargin{1,3})
                % normal mode:
                % wf_R_b = varargin{1}
                % wf_p_b = varargin{2}
                % q_j    = varargin{3}
                % v_b    = varargin{5}
                dq_j = varargin{1,4};

                [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
            else
                % optimized mode:
                % djcdq_f = varargin{2}
                Jc_f = varargin{1,1};
                M    = varargin{1,3};
                c_qv = varargin{1,4};
                dq_j = varargin{1,5};
            end
            nu = varargin{1,6};
        case 10 % optimized modes:
            % v_b = varargin{3}
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,4};

            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin{1:4});
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        case 9
            % v_b = varargin{2}
            dq_j = varargin{1,1};
            nu   = varargin{1,3};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, varargin{1:3});
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        otherwise
            error('WBM::jointAccelerationsFHPCPL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % mixed velocity of the hands at the contact links {lnk} ...
    wf_v_lnk = a_prms.Jc_h * dq_j;

    % apply velocity and acceleration saturation:
    % (to prevent overload & integration problems)
    wf_v_lnk = WBM.utilities.mbd.satVel(wf_v_lnk);
    wf_a_lnk = WBM.utilities.mbd.satAcc(wf_a_lnk);

    % calculate the payload forces of the hands in contact space {c} = {lnk}:
    f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk);

    % compute the contact forces of the hands (optimized mode):
    [fc_h,~] = contactForcesCLPCEF(obj, hand_conf, tau, f_pl, wf_a_lnk, a_prms.Jc_h, ...
                                   a_prms.djcdq_h, M, c_qv, dq_j, nu);

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the feet and hands:
    J_c   = vertcat(Jc_f, a_prms.Jc_h);
    f_c   = vertcat(a_prms.fc_f, fc_h);
    Jc_t  = J_c.';
    ddq_j = M \ (a_prms.tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        a_c = vertcat(ac_f, wf_a_lnk);
        fd_prms = struct('tau_gen', a_prms.tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_pl', f_pl);
    end
end
