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

function [ddq_j, fd_prms] = jointAccelerationsNFBPL(obj, hand_conf, tau, fhTotCWrench, f_cp, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` of a humanoid
    % robot with *payload forces* that are acting on the hands of the robot, but
    % without a *floating base* (*NFBPL*) or by exclusion of it.
    %
    % Since the given robot is defined without a floating base, there are no
    % contact forces :math:`f_c` to the ground, i.e. :math:`f_c = 0`. The
    % behavior of the model is the same as a robot with a fixed base.
    %
    % The joint accelerations :math:`\ddot{q}_j` will be calculated as defined
    % in equation :eq:`joint_accelerations`, but without the contact forces of the
    % feet. In order to obtain the joint accelerations from the robot system, the
    % method computes at first the *contact forces* :math:`f_c` of the hands, in
    % dependency of the *payload forces* that are acting on them.
    %
    % The method can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsNFBPL(hand_conf, tau, fhTotCWrench, f_cp, wf_R_b, wf_p_b, q_j, dq_j, v_b)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and the joint velocities:
    %
    %   .. py:method:: jointAccelerationsNFBPL(clnk_conf, tau, fhTotCWrench, f_cp[, M, c_qv], dq_j)
    %
    % Arguments:
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
    %        of the forward dynamics calculation with the fields ``tau_gen``, ``f_c``,
    %        ``a_c`` and ``f_pl`` (*optional*).
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerationsNFB`, :meth:`WBM.jointAccelerationsNFBEF`,
    %   :meth:`WBM.handPayloadForces` and :meth:`WBM.contactForcesEF`.
    switch nargin
        case 10
            % normal mode:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            % compute the whole body dynamics of the hands ...
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, Jc_h, djcdq_h] = wholeBodyDynamicsCS(obj, hand_conf, wf_R_b_arr, varargin{1,2}, ...
                                                           varargin{1,3}, dq_j, varargin{1,5});
        case 8 % optimized modes:
            M    = varargin{1,1};
            c_qv = varargin{1,2};
            dq_j = varargin{1,3};

            [Jc_h, djcdq_h] = contactJacobiansCS(obj, hand_conf);
        case 6
            dq_j = varargin{1,1};
            [M, c_qv, Jc_h, djcdq_h] = wholeBodyDynamicsCS(obj, hand_conf);
        otherwise
            error('WBM::jointAccelerationsNFBPL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % remove the floating base part ...
    M    = M(7:end,7:end);
    c_qv = c_qv(7:end,1);
    Jc_h = Jc_h(:,7:end);

    % get the generalized forces with friction ...
    tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
    tau_gen = tau + tau_fr;              % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                         % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix.

    % calculate the mixed accelerations of the hands at the contact links {lnk}:
    % note: since this forward dynamics model has no floating base, there
    %       are no contact forces and accelerations on the ground, i.e.
    %       Jcf_t*fc_f = 0, the same as a robot with a fixed base.
    ddq      = M \ (tau_gen - c_qv);
    wf_a_lnk = Jc_h*ddq + djcdq_h;
    % mixed velocity of the hands at the contact links {lnk} ...
    wf_v_lnk = Jc_h * dq_j;

    % apply velocity and acceleration saturation:
    % (to prevent overload & integration problems)
    wf_v_lnk = WBM.utilities.mbd.satVel(wf_v_lnk);
    wf_a_lnk = WBM.utilities.mbd.satAcc(wf_a_lnk);

    % calculate the payload forces of the hands in contact space {c} = {lnk}:
    f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk);

    % compute the contact forces of the hands (with friction):
    [fc_h,~] = contactForcesEF(obj, tau, f_pl, wf_a_lnk, Jc_h, djcdq_h, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j without floating base,
    % in dependency of the current payload forces at each contact point p_c:
    Jc_t  = Jc_h.';
    ddq_j = M \ (tau_gen + Jc_t*fc_h - c_qv);
    % ddq_j = M \ (tau_gen + Jc_t*f_pl - c_qv);
    ddq_j = vertcat(zeros(6,1), ddq_j);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', fc_h, 'a_c', wf_a_lnk, 'f_pl', f_pl);
    end
end
