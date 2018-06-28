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

function [f_c, tau_gen] = contactForcesCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc, djcdq, M, c_qv, varargin)
    % Computes the *contact forces* :math:`f_c` of the given *contact constraints*
    % with *contact link pose corrections* and additionally *external forces*
    % (*CLPCEF*) that are acting on the contact links of the robot.
    %
    % The formula for calculating the *contact constraint forces* is derived
    % from the dynamic equations of motion of the robot, such that:
    %
    %   .. math::
    %      :label: contact_forces_clpcef
    %
    %      f_c = \Lambda_c\cdot (a_c + J_c M^{\text{-}1}\cdot (C(q_j, \dot{q}_j) -
    %      \tau_{gen}) - \dot{J}_c\dot{q}_j - k_v \dot{\varepsilon}_c -
    %      k_p \varepsilon_c) - f^{\small C}_e
    %
    % The variable :math:`\Lambda_c = \Upsilon_{c}^{\text{-}1} = (J_c M^{\text{-}1} J_{c}^{T})^{\text{-}1}`
    % denotes the *inertia matrix* (or *pseudo-kinetic energy matrix*
    % :cite:`Khatib1987`) in contact space :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`,
    % :math:`\tau_{gen} = S_j\cdot (\tau - \tau_{fr})` represents the
    % *generalized forces* with the *joint selection matrix*
    % :math:`S_j = [\mathbf{0}_{(6 \times n)}\ \mathbb{I}_n]^T` and the *friction
    % forces* :math:`\tau_{fr}`. The variable :math:`a_c` in the formula denotes
    % the *mixed contact accelerations* and the variables :math:`k_p` and :math:`k_v`
    % are *stiffness* and *damping control gains* for the closed-loop system. The
    % variables :math:`\varepsilon_c` and :math:`\dot{\varepsilon}_c = J_c\nu`
    % denoting the *position* and *velocity errors* of the position-regulation
    % system with the *mixed generalized velocities* :math:`\nu`. The force vector
    % :math:`f^{\small C}_e` represents the *external forces* that are acting on
    % the *contact links* in contact space :math:`\mathrm{C}`.
    %
    % The method can be called as follows:
    %
    %   - .. py:method:: contactForcesCLPCEF(clnk_conf, tau, fe_c, ac, Jc, djcdq, M, c_qv, wf_R_b_arr, wf_p_b, q_j[, dq_j], nu)
    %   - .. py:method:: contactForcesCLPCEF(clnk_conf, tau, fe_c, ac, Jc, djcdq, M, c_qv[, dq_j], nu)
    %
    % Arguments:
    %   clnk_conf     (struct): Configuration structure to specify the *qualitative
    %                           state* of at most two *contact links*.
    %
    %                           The data structure specifies which link is currently
    %                           in contact with the ground or an object. It specifies
    %                           also the *desired poses*, *angular velocities* and
    %                           *control gains* for the position-regulation system
    %                           of the links.
    %   tau   (double, vector): :math:`(n \times 1)` torque force vector for the
    %                           joints and the base of the robot.
    %   fe_c  (double, vector): :math:`(k \times 1)` vector of external forces (in
    %                           contact space) that are affecting at the specified
    %                           contact links with size of :math:`k = 6` (one link)
    %                           or :math:`k = 12` (both links).
    %   ac    (double, vector): :math:`(k \times 1)` mixed acceleration vector for
    %                           the *contact points* of the specified contact links
    %                           with the size of :math:`k = 6` (one link) or
    %                           :math:`k = 12` (both links).
    %
    %                           **Note:** If the given accelerations are very
    %                           small, then the vector can also be *constant*
    %                           or *zero*.
    %   Jc    (double, matrix): :math:`(6 m \times n)` Jacobian of the
    %                           *contact constraints* in contact space
    %                           :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`.
    %   djcdq (double, vector): :math:`(6 m \times 1)` product vector of
    %                           the time derivative of the contact Jacobian
    %                           :math:`\dot{J}_c` and the joint velocity
    %                           :math:`\dot{q}_j`.
    %   M     (double, matrix): :math:`(n \times n)` generalized mass matrix
    %                           of the robot.
    %   c_qv  (double, vector): :math:`(n \times 1)` generalized bias force
    %                           vector of the robot.
    %   nu    (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
    %                           velocity vector (generalized base velocity and
    %                           joint velocity).
    %
    % The variable :math:`m` denotes the *number of contact constraints* of
    % the robot and :math:`n = n_{dof} + 6`.
    %
    % Other Parameters:
    %   wf_R_b_arr (double, vector): :math:`(9 \times 1)` rotation matrix reshaped
    %                                in vector form from the base frame *b* to the
    %                                world frame *wf* (*optional*).
    %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from the base
    %                            frame *b* to the world frame *wf* (*optional*).
    %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions vector
    %                            in :math:`[\si{\radian}]` (*optional*).
    %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                            vector in :math:`[\si{\radian/s}]` (*optional*).
    %
    %                            **Note:** If the joint velocity is not given,
    %                            then the method assumes that the robot system
    %                            is *frictionless*.
    % Returns:
    %   [f_c, tau_gen]: 2-element tuple containing:
    %
    %      - **f_c**     (*double, vector*) -- :math:`(n \times 1)` contact
    %        constraint force vector,
    %      - **tau_gen** (*double, vector*) -- :math:`(n \times 1)` generalized
    %        force vector of the robot,
    %
    %   with :math:`n = n_{dof} + 6`.
    %
    % See Also:
    %   :meth:`WBM.contactForces`, :meth:`contactForcesEF` and
    %   :meth:`WBMBase.frictionForces`.
    %
    % References:
    %   - :cite:`Park2006`, chapter 5, pp. 106-110, eq. (5.5)-(5.14).
    %   - :cite:`Murray1994`, pp. 269-270, eq. (6.5) and (6.6).
    %   - :cite:`Khatib1987`, p. 49-50, eq. (51).

    % References:
    %   [Par06] Park, Jaeheung: Control Strategies for Robots in Contact.
    %           PhD-Thesis, Artificial Intelligence Laboratory, Stanford University,
    %           2006, Chapter 5, pp. 106-110, eq. (5.5)-(5.14),
    %           URL: <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
    %   [MLS94] Murray, R. M.; Li, Z.; Sastry, S. S.: A Mathematical Introduction to Robotic Manipulation.
    %           CRC Press, 1994, pp. 269-270, eq. (6.5) and (6.6).
    %   [Kha87] Khatib, Oussama: A unified approach for motion and force control of robot manipulators: The operational space formulation.
    %           In: IEEE Journal on Robotics and Automation, Volume 3, Issue 1, 1987, p. 49-50, eq. (51),
    %           URL: <https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1987_RA.pdf>.
    switch nargin
        % fe_c ... external forces that are acting on the contact links (in contact space)
        % ac   ... mixed contact accelerations
        case 14 % normal modes:
            % generalized forces with friction:
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            dq_j = varargin{1,4};
            nu   = varargin{1,5}; % mixed generalized velocity

            tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
            tau_gen = tau + tau_fr;              % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                 % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix
            perror_cl = poseErrorCL(obj, clnk_conf, varargin{1:3});
        case 13
            % without friction:
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            nu = varargin{1,4};

            tau_gen   = tau;
            perror_cl = poseErrorCL(obj, clnk_conf, varargin{1:3});
        case 11 % optimized modes:
            % with friction:
            dq_j = varargin{1,1};
            nu   = varargin{1,2};

            tau_fr    = frictionForces(obj, dq_j);
            tau_gen   = tau + tau_fr;
            perror_cl = poseErrorCL(obj, clnk_conf);
        case 10
            % without friction:
            nu = varargin{1,1};

            tau_gen   = tau;
            perror_cl = poseErrorCL(obj, clnk_conf);
        otherwise
            error('WBM::contactForcesCLPCEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    if (size(tau,1) < size(c_qv,1))
        tau_gen = vertcat(zeros(6,1), tau_gen);
    end
    if ( isscalar(perror_cl) && ~perror_cl )
        % both contact links have no contact to the ground/object ...
        f_c = obj.ZERO_CVEC_12;
        return
    end
    k_p = clnk_conf.ctrl_gains.k_p; % control gain for correcting the link positions (position feedback).
    k_v = clnk_conf.ctrl_gains.k_v; % control gain for correcting the velocities (rate feedback).

    % Calculation of the contact force vector for a closed-loop control system with additional
    % velocity and position correction for the contact links (position-regulation system):
    Jc_t      = Jc.';
    JcMinv    = Jc / M; % = Jc * M^(-1)
    Upsilon_c = JcMinv * Jc_t; % inverse mass matrix Upsilon_c = Lambda^(-1) = Jc * M^(-1) * Jc^T in contact space {c},
                               % Lambda^(-1) ... inverse pseudo-kinetic energy matrix.
    % contact constraint forces (generated by the environment) ...
    f_c = (Upsilon_c \ (ac + JcMinv*(c_qv - tau_gen) - djcdq - k_v.*(Jc*nu) - k_p.*perror_cl)) - fe_c;
    % (this calculation method is numerically more accurate and robust than the calculation variant with the cartmass-function.)
end
%% END of contactForcesCLPCEF.


%% POSE TRANSFORMATIONS & POSE ERROR FUNCTIONS:

function perror_cl = poseErrorCL(obj, clnk_conf, varargin)
    switch clnk_conf.rtype
        case 'e'
            % use Euler-angles:
            perror_cl = poseErrorCLE(obj, clnk_conf, varargin{:});
        case 'q'
            % use quaternions:
            perror_cl = poseErrorCLQ(obj, clnk_conf, varargin{:});
        otherwise
            error('poseErrorCL: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
end

function perror_cl = poseErrorCLE(obj, clnk_conf, varargin) % via Euler-angles
    ctc_l = clnk_conf.contact.left;
    ctc_r = clnk_conf.contact.right;

    % check which link is in contact with the ground or object and calculate the corresponding
    % error between the reference (desired) and the new link transformations:
    if (ctc_l && ctc_r)
        % both links are in contact with the ground/object:
        clink_l = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_l};
        clink_r = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_r};

        % set the desired poses (VE-Transformations (i)) of the contact links as reference ...
        fk_ref_pose.veT_llnk = clnk_conf.des_pose.veT_llnk;
        fk_ref_pose.veT_rlnk = clnk_conf.des_pose.veT_rlnk;

        % get the new VQ-transformations (link frames) of the contact links ...
        fk_new_pose = poseTransformationsCL(clink_l, clink_r, varargin{:});
        % convert the link frames in VE-Transformations (veT) ...
        [p_ll, eul_ll] = WBM.utilities.tfms.frame2posEul(fk_new_pose.vqT_llnk);
        [p_rl, eul_rl] = WBM.utilities.tfms.frame2posEul(fk_new_pose.vqT_rlnk);
        fk_new_pose.veT_llnk = vertcat(p_ll, eul_ll); % current motions
        fk_new_pose.veT_rlnk = vertcat(p_rl, eul_rl);

        % compute the Euler angle velocity transformations ...
        Er_ll = WBM.utilities.tfms.eul2angVelTF(eul_ll);
        Er_rl = WBM.utilities.tfms.eul2angVelTF(eul_rl);
        % create for each link the mixed velocity transformation matrix ...
        vX_ll = WBM.utilities.tfms.mixveltfm(Er_ll);
        vX_rl = WBM.utilities.tfms.mixveltfm(Er_rl);

        % get the pose error (distances) between the contact link poses (CLP):
        % delta  =   vX * (current transf. T   -   desired transf. T*)
        %                   (curr. motion)           (ref. motion)
        delta_ll = vX_ll*(fk_new_pose.veT_llnk - fk_ref_pose.veT_llnk);
        delta_rl = vX_rl*(fk_new_pose.veT_rlnk - fk_ref_pose.veT_rlnk);

        perror_cl = vertcat(delta_ll, delta_rl);
    elseif ctc_l
        % only the left link is in contact with the ground/object:
        clink_l = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_l};

        % set the desired pose transformation as reference and
        % compute the new pose transformation:
        fk_ref_pose.veT_llnk = clnk_conf.des_pose.veT_llnk;
        fk_new_pose = poseTransformationLeftCL(clink_l, varargin{:});
        % convert to VE-transformation ...
        [p_ll, eul_ll] = WBM.utilities.tfms.frame2posEul(fk_new_pose.vqT_llnk);
        fk_new_pose.veT_llnk = vertcat(p_ll, eul_ll); % current motion

        % create the mixed velocity transformation ...
        Er_ll = WBM.utilities.tfms.eul2angVelTF(eul_ll);
        vX_ll = WBM.utilities.tfms.mixveltfm(Er_ll);
        % compute the pose error (distance) ...
        perror_cl = vX_ll*(fk_new_pose.veT_llnk - fk_ref_pose.veT_llnk);
    elseif ctc_r
        % only the right link is in contact with the ground/object:
        clink_r = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_r};

        % set the desired pose transformation as reference and
        % compute the new pose transformation:
        fk_ref_pose.veT_rlnk = clnk_conf.des_pose.veT_rlnk;
        fk_new_pose = poseTransformationRightCL(clink_r, varargin{:});

        [p_rl, eul_rl] = WBM.utilities.tfms.frame2posEul(fk_new_pose.vqT_rlnk);
        fk_new_pose.veT_rlnk = vertcat(p_rl, eul_rl);

        % create the mixed velocity transformation ...
        Er_rl = WBM.utilities.tfms.eul2angVelTF(eul_rl);
        vX_rl = WBM.utilities.tfms.mixveltfm(Er_rl);
        % compute the pose error (distance) ...
        perror_cl = vX_rl*(fk_new_pose.veT_rlnk - fk_ref_pose.veT_rlnk);
    else
        % both links have no contact to the ground/object ...
        perror_cl = 0;
    end
end

% (i) veT: Position vector with Euler angles (in this case it represents a
%     joint motion m(t) = (p(t), e(t))^T, where p(t) in R^3 and e(t) in S^3).

function perror_cl = poseErrorCLQ(obj, clnk_conf, varargin) % via quaternions
    ctc_l = clnk_conf.contact.left;
    ctc_r = clnk_conf.contact.right;

    % check which link is in contact with the ground or object and calculate the corresponding
    % error between the reference (desired) and the new link transformations:
    if (ctc_l && ctc_r)
        % both links are in contact with the ground/object:
        clink_l = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_l};
        clink_r = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_r};

        % set the desired poses (VQ-Transformations (ii)) of the contact links as reference ...
        fk_ref_pose.vqT_llnk = clnk_conf.des_pose.vqT_llnk;
        fk_ref_pose.vqT_rlnk = clnk_conf.des_pose.vqT_rlnk;

        % get the new VQ-transformations (link frames) and the corresponding
        % orientations of the contact links ...
        fk_new_pose = poseTransformationsCL(clink_l, clink_r, varargin{:});
        quat_ll = fk_new_pose.vqT_llnk(4:7,1);
        quat_rl = fk_new_pose.vqT_rlnk(4:7,1);

        % compute the quaternion velocity transformations ...
        Er_ll = WBM.utilities.tfms.quat2angVelTF(quat_ll);
        Er_rl = WBM.utilities.tfms.quat2angVelTF(quat_rl);
        % create for each link the mixed velocity transformation matrix ...
        vX_ll = WBM.utilities.tfms.mixveltfm(Er_ll);
        vX_rl = WBM.utilities.tfms.mixveltfm(Er_rl);

        % get the pose error (distances) between the contact link poses (CLP):
        % delta  =   vX * (current transf. T   -   desired transf. T*)
        %                   (curr. motion)           (ref. motion)
        delta_ll = vX_ll*(fk_new_pose.vqT_llnk - fk_ref_pose.vqT_llnk);
        delta_rl = vX_rl*(fk_new_pose.vqT_rlnk - fk_ref_pose.vqT_rlnk);

        perror_cl = vertcat(delta_ll, delta_rl);
    elseif ctc_l
        % only the left link is in contact with the ground/object:
        clink_l = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_l};

        % set the desired pose transformation as reference and compute
        % the new pose transformation with the corresponding orientation:
        fk_ref_pose.vqT_llnk = clnk_conf.des_pose.vqT_llnk;
        fk_new_pose = poseTransformationLeftCL(clink_l, varargin{:});
        quat_ll = fk_new_pose.vqT_llnk(4:7,1);

        % create the mixed velocity transformation ...
        Er_ll = WBM.utilities.tfms.quat2angVelTF(quat_ll);
        vX_ll = WBM.utilities.tfms.mixveltfm(Er_ll);
        % compute the pose error (distance) ...
        perror_cl = vX_ll*(fk_new_pose.vqT_llnk - fk_ref_pose.vqT_llnk);
    elseif ctc_r
        % only the right link is in contact with the ground/object:
        clink_r = obj.mwbm_config.ccstr_link_names{1,clnk_conf.lnk_idx_r};

        % set the desired pose transformation as reference and compute
        % the new pose transformation with the corresponding orientation:
        fk_ref_pose.vqT_rlnk = clnk_conf.des_pose.vqT_rlnk;
        fk_new_pose = poseTransformationRightCL(clink_r, varargin{:});
        quat_rl = fk_new_pose.vqT_rlnk(4:7,1);

        % create the mixed velocity transformation ...
        Er_rl = WBM.utilities.tfms.quat2angVelTF(quat_rl);
        vX_rl = WBM.utilities.tfms.mixveltfm(Er_rl);
        % compute the pose error (distance) ...
        perror_cl = vX_rl*(fk_new_pose.vqT_rlnk - fk_ref_pose.vqT_rlnk);
    else
        % both links have no contact to the ground/object ...
        perror_cl = 0;
    end
end

% (ii) vqT: Vector quaternion transformation (VQT). Position vector with a quaternion
%      as orientation (in this case it represents a joint motion m(t) = (p(t), q(t))^T,
%      where p(t) in R^3 and q(t) in R^4).

function fk_new_pose = poseTransformationsCL(clink_l, clink_r, varargin)
    % get the new positions and orientations (VQ-transformations) for both contact links:
    switch nargin
        case 5 % normal mode:
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            fk_new_pose.vqT_llnk = mexWholeBodyModel('forward-kinematics', varargin{1,1}, varargin{1,2}, varargin{1,3}, clink_l);
            fk_new_pose.vqT_rlnk = mexWholeBodyModel('forward-kinematics', varargin{1,1}, varargin{1,2}, varargin{1,3}, clink_r);
        case 2 % optimized mode:
            fk_new_pose.vqT_llnk = mexWholeBodyModel('forward-kinematics', clink_l);
            fk_new_pose.vqT_rlnk = mexWholeBodyModel('forward-kinematics', clink_r);
        otherwise
            error('poseTransformationsCL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end

function fk_new_pose = poseTransformationLeftCL(clink_l, varargin)
    % get the new VQ-transformation for the left contact link:
    switch nargin
        case 4 % normal mode:
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            fk_new_pose.vqT_llnk = mexWholeBodyModel('forward-kinematics', varargin{1,1}, varargin{1,2}, varargin{1,3}, clink_l);
        case 1 % optimized mode:
            fk_new_pose.vqT_llnk = mexWholeBodyModel('forward-kinematics', clink_l);
        otherwise
            error('poseTransformationLeftCL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end

function fk_new_pose = poseTransformationRightCL(clink_r, varargin)
    % get the new VQ-transformation for the right contact link:
    switch nargin
        case 4 % normal mode:
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            fk_new_pose.vqT_rlnk = mexWholeBodyModel('forward-kinematics', varargin{1,1}, varargin{1,2}, varargin{1,3}, clink_r);
        case 1 % optimized mode:
            fk_new_pose.vqT_rlnk = mexWholeBodyModel('forward-kinematics', clink_r);
        otherwise
            error('poseTransformationRightCL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
