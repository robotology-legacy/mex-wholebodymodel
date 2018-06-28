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

function [ddq_j, fd_prms] = jointAccelerationsCLPCEF(obj, clnk_conf, tau, fe_c, ac, varargin)
    % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` with *contact
    % link pose corrections* and additionally *external forces* (*CLPCEF*) that
    % are acting on the specified contact links of the floating-base robot.
    %
    % The joint accelerations :math:`\ddot{q}_j` will be calculated as defined
    % in equation :eq:`joint_accelerations`. For a *closed-loop control system*
    % with *velocity* and *position regulation*, the calculated joint
    % acceleration vector depends on the given *contact constraints* and on the
    % additional *external forces* that are acting on the contact links of the
    % floating-base robot.
    %
    % The method assumes that *only the contact links* of the robot may have
    % contact with the *ground* or an *object* and can be called in three
    % different modes:
    %
    % **Normal mode** -- Computes the joint accelerations, specified by
    % by the base orientation, the positions and the velocities:
    %
    %   .. py:method:: jointAccelerationsCLPCEF(clnk_conf, tau, fe_c, ac, wf_R_b, wf_p_b, q_j, dq_j, v_b, nu)
    %
    % **Semi-optimized mode:**
    %
    %   .. py:method:: jointAccelerationsCLPCEF(clnk_conf, tau, fe_c, ac, wf_R_b, wf_p_b, q_j, nu)
    %
    % **Optimized mode** -- Computes the joint accelerations at the current
    % state of the robot system, in dependency of the given whole-body
    % dynamics and velocities:
    %
    %   - .. py:method:: jointAccelerationsCLPCEF(clnk_conf, tau, fe_c, ac, Jc, djcdq, M, c_qv[, dq_j], nu)
    %   - .. py:method:: jointAccelerationsCLPCEF(clnk_conf, tau, fe_c, ac, nu)
    %
    % Arguments:
    %   clnk_conf         (struct): Configuration structure to specify the *qualitative
    %                               state* of at most two *contact links*.
    %
    %                               The data structure specifies which link is currently
    %                               in contact with the ground or an object. It specifies
    %                               also the *desired poses*, *angular velocities* and
    %                               *control gains* for the position-regulation system
    %                               of the links.
    %   tau       (double, vector): :math:`(n \times 1)` torque force vector for
    %                               the joints and the base of the robot with
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
    %                               vector in :math:`[\si{\radian/s}]` (*optional*).
    %   v_b       (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                               vector (*optional*).
    %   nu        (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
    %                               velocity vector (generalized base velocity and
    %                               joint velocity).
    %
    % The variable :math:`k` indicates the *size* of the given *force* and
    % *acceleration vectors* in dependency of the specified contact links:
    %
    %   - :math:`k = 6`  -- only one link is defined.
    %   - :math:`k = 12` -- both links are defined.
    %
    % If the joint velocity vector :math:`\dot{q}_j` (``dq_j``) is not given
    % as an argument, then the method assumes that the robot system is
    % *frictionless*.
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
    %   :meth:`WBM.jointAccelerationsFPC`, :meth:`WBM.jointAccelerationsHPCEF` and
    %   :meth:`WBM.contactForcesCLPCEF`.
    switch nargin
        case 11
            if iscolumn(varargin{1,3})
                % normal mode (generalized forces with friction):
                % wf_R_b = varargin{1}
                wf_p_b = varargin{1,2};
                q_j    = varargin{1,3};
                dq_j   = varargin{1,4};
                v_b    = varargin{1,5};
                nu     = varargin{1,6};

                % compute the whole body dynamics and for each contact constraint
                % the Jacobian and the product of the Jacobian derivative ...
                wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clnk_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                % get the contact forces and the corresponding generalized forces ...
                [f_c, tau_gen] = contactForcesCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc, djcdq, M, c_qv, ...
                                                     wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, nu);
            else
                % optimized mode (with friction):
                % djcdq = varargin{2}
                % dq_j  = varargin{5}
                % nu    = varargin{6}
                Jc   = varargin{1,1};
                M    = varargin{1,3};
                c_qv = varargin{1,4};

                [f_c, tau_gen] = contactForcesCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc, ...
                                                     varargin{1,2}, M, c_qv, varargin{5:6});
            end
        case 10
            % optimized mode (without friction):
            % djcdq = varargin{2}
            % nu    = varargin{5}
            Jc   = varargin{1,1};
            M    = varargin{1,3};
            c_qv = varargin{1,4};

            [f_c, tau_gen] = contactForcesCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc, ...
                                                 varargin{1,2}, M, c_qv, varargin{1,5});
        case 9
            % semi-optimized mode (without friction):
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % nu     = varargin{4}
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clnk_conf);
            [f_c, tau_gen] = contactForcesCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc, djcdq, ...
                                                 M, c_qv, wf_R_b_arr, varargin{2:4});
        case 6
            % optimized mode (without friction):
            nu = varargin{1,1};

            [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clnk_conf);
            [f_c, tau_gen] = contactForcesCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc, djcdq, M, c_qv, nu);
        otherwise
            error('WBM::jointAccelerationsCLPCEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end

    % Joint Acceleration q_ddot (derived from the dyn. equations of motion):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    Jc_t  = Jc.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (...)

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'a_c', ac, 'f_e', fe_c);
    end
end
