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

function [ddq_j, fd_prms] = jointAccelerationsNFBEF(obj, clnk_conf, tau, fe_c, ac, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` of a humanoid
    % robot with *external forces* that are acting on the contact links of the
    % robot, but without a *floating base* (*NFBEF*) or by exclusion of it --
    % *experimental*.
    %
    % Since the given robot is defined without a floating base, there are no
    % contact forces :math:`f_c` to the ground, i.e. :math:`f_c = 0`. The
    % behavior of the model is the same as a robot with a fixed base.
    %
    % The joint accelerations :math:`\ddot{q}_j` will be calculated as defined
    % in equation :eq:`joint_accelerations`, but without the contact forces of the
    % feet. In order to obtain the joint accelerations from the robot system, the
    % method computes at first the *contact forces* :math:`f_c` of the contact
    % links, in dependency of the *external forces* that are acting on them.
    %
    % The method can be called in two different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsNFBEF(clnk_conf, tau, fe_c, ac, wf_R_b, wf_p_b, q_j, dq_j, v_b)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and the joint velocities:
    %
    %   .. py:method:: jointAccelerationsNFBEF(clnk_conf, tau, fe_c, ac[, M, c_qv], dq_j)
    %
    % Note:
    %   This method is still untested and should be considered as *experimental*.
    %
    % Arguments:
    %   clnk_conf         (struct): Configuration structure to specify the *qualitative
    %                               state* of at most two *contact links*.
    %
    %                               The data structure specifies which link is currently
    %                               in contact with an object or a wall.
    %   tau       (double, vector): :math:`(n \times 1)` torque force vector for the
    %                               joints and the base of the robot with
    %                               :math:`n = n_{dof} + 6`.
    %   fe_c      (double, vector): :math:`(k \times 1)` vector of external forces
    %                               (in contact space :math:`\mathrm{C}`) that are
    %                               acting on the specified contact links.
    %
    %                               **Note:** The *external forces* are either *constant*
    %                               or *zero*.
    %   ac        (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                               for the *contact points* of the contact links.
    %
    %                               **Note:** If the accelerations are very small,
    %                               then the vector can also be *constant* or *zero*.
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
    %
    % The variable :math:`k` indicates the *size* of the given *force* and
    % *acceleration vectors* in dependency of the specified contact links:
    %
    %   - :math:`k = 6`  -- only one link is defined.
    %   - :math:`k = 12` -- both links are defined.
    %
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
    %        of the forward dynamics calculation with the fields ``tau_gen``, ``f_c``
    %        and ``f_e`` (*optional*).
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerationsNFB`, :meth:`WBM.jointAccelerationsNFBPL` and
    %   :meth:`WBM.contactForcesEF`.
    switch nargin
        case 10
            % normal mode:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            % compute the whole body dynamics of each contact constraint
            % of the given contact links configuration ...
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, J_c, djcdq] = wholeBodyDynamicsCS(obj, clnk_conf, wf_R_b_arr, varargin{1,2}, ...
                                                        varargin{1,3}, dq_j, varargin{1,5});
        case 8 % optimized modes:
            M    = varargin{1,1};
            c_qv = varargin{1,2};
            dq_j = varargin{1,3};

            [J_c,~] = contactJacobiansCS(obj, clnk_conf);
        case 6
            dq_j = varargin{1,1};
            [M, c_qv, J_c, djcdq] = wholeBodyDynamicsCS(obj, clink_conf);
        otherwise
            error('WBM::jointAccelerationsNFBEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % remove the floating base part ...
    M    = M(7:end,7:end);
    c_qv = c_qv(7:end,1);
    J_c  = J_c(:,7:end);

    % % get the generalized forces with friction ...
    % tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
    % tau_gen = tau + tau_fr;

    % contact forces and generalized forces (with friction):
    [f_c, tau_gen] = contactForcesEF(obj, tau, fe_c, ac, J_c, djcdq, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j without
    % floating base and in dependency of the external forces f_e:
    Jc_t  = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv);
    % ddq_j = M \ (tau_gen - Jc_t*fe_c - c_qv);
    ddq_j = vertcat(zeros(6,1), ddq_j);

    if (nargout == 2)
        % set the forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'f_e', fe_c);
    end
end
