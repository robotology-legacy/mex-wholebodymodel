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

function clnk_conf = clnkConfigState(obj, varargin)
    % Creates the *contact links configuration* in dependency of the given
    % *contact state* of the contact links.
    %
    % The method creates a data structure that specifies which contact link of
    % the robot is currently in "contact" with the environment. If required, the
    % structure specifies also the *desired poses*, *angular velocities* and
    % the corresponding *control gains* for the position-regulation system of
    % the contact links.
    %
    % Note:
    %   The *contact links* are **not** representing the "contact points" where
    %   the robot is in contact with the environment. The contact links are
    %   those links of the robot that are "behind" the contact points.
    %
    % The method can be called as follows:
    %
    %   - .. py:method:: clnkConfigState(cstate, clnk_idx, vqT_lnk, k_p, k_v[, rtype])
    %   - .. py:method:: clnkConfigState(cstate, clnk_idx, veT_lnk, k_p, k_v[, rtype])
    %   - .. py:method:: clnkConfigState(cstate, clnk_idx, q_j, k_p[, rtype])
    %   - .. py:method:: clnkConfigState(cstate, clnk_idx, [, q_j[, rtype]])
    %   - .. py:method:: clnkConfigState(clnk_idx)
    %
    % Arguments:
    %   cstate (logical, vector): :math:`(1 \times 2)` boolean vector pair to
    %                             indicate which contact link is in contact with
    %                             the environment (ground, object or wall).
    %
    %                             First element:
    %                                Contact state of the *left contact link*.
    %
    %                             Second element:
    %                                Contact state of the *right contact link*.
    %
    %                             A contact link touches the ground if the
    %                             corresponding value is set to *true*,
    %                             otherwise *false*.
    %   vqT_lnk (double, vector): :math:`(7 \times 1)` VQ-transformation frame
    %                             (vector-quaternion frame) relative from the
    %                             given link frame *lnk* to the world frame *wf*
    %                             (*optional*).
    %   veT_lnk (double, vector): :math:`(6 \times 1)` VE-transformation frame
    %                             (vector-Euler frame) relative from the given
    %                             link frame *lnk* to the world frame *wf*
    %                             (*optional*).
    %   k_p     (double, scalar): Stiffness control gain for the closed-loop system
    %                             (*optional*).
    %   k_v     (double, scalar): Damping control gain for the closed-loop system
    %                             (*optional*).
    %
    %                             **Note:** If the control gain is not defined, then
    %                             by default the method computes the gain value for
    %                             *critical damping* with :math:`k_v = 2\cdot \sqrt{k_p}`.
    %   rtype     (char, vector): Specifies the *rotation representation type* for the
    %                             desired poses of the links as reference. The rotation
    %                             can be represented either in quaternions ``quat`` or
    %                             in Euler-angles ``eul`` (default type: ``eul``).
    % Returns:
    %   clnk_conf (struct): Configuration structure that specifies the current
    %   *qualitative state* of the contact links.
    tf_data = false;

    switch nargin
        case 7
            cstate   = varargin{1,1};
            clnk_idx = varargin{1,2};
            k_p      = varargin{1,4}; % stiffness gain k_p for the closed-loop system (1)
            k_v      = varargin{1,5}; % damping gain k_v for the closed-loop system (2)
            rtype    = varargin{1,6}; % rotation representation type

            ndof = obj.mwbm_model.ndof;
            v    = varargin{1,3};
            [vlen, nTfms] = size(v);
            switch nTfms
                case 1
                    % v is column-vector:
                    switch vlen
                        case ndof
                            q_j = v;
                        case 7
                            % using quaternions:
                            vqT_lnk = v;
                            tf_data = true;
                        case 6
                            % using Euler-angles:
                            veT_lnk = v;
                            tf_data = true;
                        otherwise
                            error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                    end
                case 2
                    % v is 2-column matrix:
                    switch vlen
                        case 7
                            % quaternions:
                            vqT_lnk = v;
                        case 6
                            % Euler-angles:
                            veT_lnk = v;
                        otherwise
                            error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                    end
                    tf_data = true;
                otherwise
                    error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            checkCState(cstate);
            checkGainValues(k_p, k_v);
        case 6
            cstate   = varargin{1,1};
            clnk_idx = varargin{1,2};
            q_j      = varargin{1,3};
            k_p      = varargin{1,4}; % (1)
            rtype    = varargin{1,5};

            checkGainValue(k_p);
            checkCState(cstate);
            % set k_v for critical damping (3)
            % Source: Craig, John J.: Introduction to Robotics: Mechanics and Control. 3rd Edition,
            %         Pearson/Prentice Hall, 2005, pp. 271-274, eq. (9.47).
            k_v = 2*sqrt(k_p);
        case 5
            cstate   = varargin{1,1};
            clnk_idx = varargin{1,2};
            q_j      = varargin{1,3};
            rtype    = varargin{1,4};

            checkCState(cstate);

            k_p = obj.DF_STIFFNESS; % use the default stiffness value ...
            k_v = 2*sqrt(k_p); % (3)
        case 3
            cstate   = varargin{1,1};
            clnk_idx = varargin{1,2};

            checkCState(cstate);
            nLnks = getLinkNbr(clnk_idx);

            clnk_conf.contact.left  = cstate(1,1);
            clnk_conf.contact.right = cstate(1,2);

            clnk_conf = setContactLinks(clnk_conf, clnk_idx, nLnks);
            return
        case 2
            clnk_idx = varargin{1,1};

            clnk_conf.contact.left  = false;
            clnk_conf.contact.right = false;

            nLnks = getLinkNbr(clnk_idx);
            clnk_conf = setContactLinks(clnk_conf, clnk_idx, nLnks);
            return
        otherwise
            error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    nLnks = getLinkNbr(clnk_idx);

    %% Setup the configuration structure for the
    %  qualitative state of the contact links:

    % specify which link is in contact with the ground/object ...
    clnk_conf.contact.left  = cstate(1,1);
    clnk_conf.contact.right = cstate(1,2);

    % set the corresponding contact links to each contact ...
    clnk_conf = setContactLinks(clnk_conf, clnk_idx, nLnks);

    % reference poses:
    if tf_data
        % verify the number of transformations ...
        if (nTfms ~= nLnks)
            error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
        end

        % set the desired poses of the links as reference ...
        switch rtype
            case 'quat'
                % using quaternions:
                clnk_conf = setDesiredRefPoseQuat(obj, clnk_conf, nLnks, vqT_lnk);
            case 'eul'
                % using Euler-angles:
                clnk_conf = setDesiredRefPoseEul(obj, clnk_conf, nLnks, veT_lnk);
            otherwise
                error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
        end
    else
        % calculate the desired poses ...
        stFltb = getFloatingBaseState(obj);
        wf_R_b_arr = reshape(stFltb.wf_R_b, 9, 1);

        switch rtype
            case 'quat'
                % quaternions:
                clnk_conf = setDesiredRefPoseQuat(obj, clnk_conf, nLnks, clnk_idx, ...
                                                   wf_R_b_arr, stFltb.wf_p_b, q_j);
            case 'eul'
                % Euler-angles:
                clnk_conf = setDesiredRefPoseEul(obj, clnk_conf, nLnks, clnk_idx, ...
                                                  wf_R_b_arr, stFltb.wf_p_b, q_j);
            otherwise
                error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
        end
    end
    % set the correction values (gains) for the
    % links to avoid numerical integration errors:
    clnk_conf.ctrl_gains.k_p = k_p; % control gain for the link positions.
    clnk_conf.ctrl_gains.k_v = k_v; % control gain for the link velocities (linear & angular).
end
%% END of clnkConfigState.


%% CONTACT LINKS, DESIRED REFERENCE POSES, CHECK CSTATE, GAINS & VE/VQ-TRANSFORMATIONS:

function clnk_conf = setContactLinks(clnk_conf, clnk_idx, nLnks)
    if (nLnks == 2)
        % both contact links (in independence
        % of the contact state condition):
        clnk_conf.lnk_idx_l = clnk_idx(1,1);
        clnk_conf.lnk_idx_r = clnk_idx(1,2);
    elseif ( nLnks && clnk_conf.contact.left ) % nLnks = 1
        % only left contact link:
        clnk_conf.lnk_idx_l = clnk_idx;
    elseif ( nLnks && clnk_conf.contact.right ) % nLnks = 1
        % only right contact link:
        clnk_conf.lnk_idx_r = clnk_idx;
    else
        % both contact links are not in contact with any surface ...
        error('setContactLinks: %s', WBM.wbmErrorMsg.NO_LNK_IN_CTC);
    end
end

function clnk_conf = setDesiredRefPoseEul(obj, clnk_conf, nLnks, varargin)
    switch nargin
        case 7
            clnk_idx   = varargin{1,1};
            wf_R_b_arr = varargin{1,2};
            wf_p_b     = varargin{1,3};
            q_j        = varargin{1,4};
        case 4
            veT_lnk    = varargin{1,1};
        otherwise
            error('setDesiredRefPoseEul: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end

    if (nLnks == 2)
        % both contact links (in independence of the contact state condition):
        % set the desired pose of the links as reference ...
        if (nargin == 4)
            % set the given VE-transformations (veT) and Euler-angles ...
            clnk_conf.des_pose.veT_llnk = veT_lnk(1:6,1); % left contact link
            clnk_conf.des_pose.veT_rlnk = veT_lnk(1:6,2); % right contact link
            eul_ll = veT_lnk(4:6,1);
            eul_rl = veT_lnk(4:6,2);
        else
            % calculate the desired poses (using veT) ...
            [clnk_conf.des_pose.veT_llnk, eul_ll] = calcDesiredRefPoseEul(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx(1,1)); % left link
            [clnk_conf.des_pose.veT_rlnk, eul_rl] = calcDesiredRefPoseEul(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx(1,2)); % right link
        end
        % set the angular velocity transformations for the contact links ...
        clnk_conf.des_pose.avT_llnk = WBM.utilities.tfms.eul2angVelTF(eul_ll);
        clnk_conf.des_pose.avT_rlnk = WBM.utilities.tfms.eul2angVelTF(eul_rl);
    elseif ( nLnks && clnk_conf.contact.left ) % nLnks = 1
        % only left contact link:
        if (nargin == 4)
            % set the given VE-transformation and Euler-angle ...
            clnk_conf.des_pose.veT_llnk = veT_lnk;
            eul_ll = veT_lnk(4:6,1);
        else
            % calculate desired pose (veT) ...
            [clnk_conf.des_pose.veT_llnk, eul_ll] = calcDesiredRefPoseEul(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx);
        end
        % set the angular velocity transformation (avT) ...
        clnk_conf.des_pose.avT_llnk = WBM.utilities.tfms.eul2angVelTF(eul_ll);
    elseif ( nLnks && clnk_conf.contact.right ) % nLnks = 1
        % only right contact link:
        if (nargin == 4)
            clnk_conf.des_pose.veT_rlnk = veT_lnk;
            eul_rl = veT_lnk(4:6,1);
        else
            [clnk_conf.des_pose.veT_rlnk, eul_rl] = calcDesiredRefPoseEul(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx);
        end
        clnk_conf.des_pose.avT_rlnk = WBM.utilities.tfms.eul2angVelTF(eul_rl);
    else
        % both contact links are not in contact with any surface ...
        error('setDesiredRefPoseEul: %s', WBM.wbmErrorMsg.NO_LNK_IN_CTC);
    end
    % set rotation type ...
    clnk_conf.rtype = 'e';
end

function clnk_conf = setDesiredRefPoseQuat(obj, clnk_conf, nLnks, varargin)
    switch nargin
        case 7
            clnk_idx   = varargin{1,1};
            wf_R_b_arr = varargin{1,2};
            wf_p_b     = varargin{1,3};
            q_j        = varargin{1,4};
        case 4
            veT_lnk    = varargin{1,1};
        otherwise
            error('setDesiredRefPoseQuat: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end

    if (nLnks == 2)
        % both contact links (in independence of the contact state condition):
        % set the desired pose of the links as reference ...
        if (nargin == 4)
            % set the given VQ-transformations (vqT) and quaternions ...
            clnk_conf.des_pose.vqT_llnk = vqT_lnk(1:7,1); % left contact link
            clnk_conf.des_pose.vqT_rlnk = vqT_lnk(1:7,2); % right contact link
            quat_ll = veT_lnk(4:7,1);
            quat_rl = veT_lnk(4:7,2);
        else
            % calculate the desired poses (using vqT) ...
            [clnk_conf.des_pose.vqT_llnk, quat_ll] = calcDesiredRefPoseQuat(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx(1,1)); % left link
            [clnk_conf.des_pose.vqT_rlnk, quat_rl] = calcDesiredRefPoseQuat(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx(1,2)); % right link
        end
        % set the angular velocity transformations for the contact links ...
        clnk_conf.des_pose.avT_llnk = WBM.utilities.tfms.quat2angVelTF(quat_ll);
        clnk_conf.des_pose.avT_rlnk = WBM.utilities.tfms.quat2angVelTF(quat_rl);
    elseif ( nLnks && clnk_conf.contact.left ) % nLnks = 1
        % only left contact link:
        if (nargin == 4)
            % set the given VQ-transformation and quaternion ...
            clnk_conf.des_pose.vqT_llnk = vqT_lnk;
            quat_ll = vqT_lnk(4:7,1);
        else
            % calculate desired pose (vqT) ...
            [clnk_conf.des_pose.vqT_llnk, quat_ll] = calcDesiredRefPoseQuat(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx);
        end
        % set the angular velocity transformation (avT) ...
        clnk_conf.des_pose.avT_llnk = WBM.utilities.tfms.quat2angVelTF(quat_ll);
    elseif ( nLnks && clnk_conf.contact.right ) % nLnks = 1
        % only right contact link:
        if (nargin == 4)
            clnk_conf.des_pose.vqT_rlnk = vqT_lnk;
            quat_rl = vqT_lnk(4:7,1);
        else
            [clnk_conf.des_pose.vqT_rlnk, quat_rl] = calcDesiredRefPoseQuat(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx);
        end
        clnk_conf.des_pose.avT_rlnk = WBM.utilities.tfms.eul2angVelTF(quat_rl);
    else
        % both contact links are not in contact with any surface ...
        error('setDesiredRefPoseQuat: %s', WBM.wbmErrorMsg.NO_LNK_IN_CTC);
    end
    % set rotation type ...
    clnk_conf.rtype = 'q';
end

function [veT_cl, eul_cl] = calcDesiredRefPoseEul(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx)
    % calculate the desired (reference) pose for the contact link in dependency
    % of the current floating-base state:
    clnk_name = obj.mwbm_config.ccstr_link_names{1,clnk_idx};

    % get the forward kinematic transformation (clink-to-world) of the contact link ...
    vqT_cl = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, clnk_name);
    % get the position and transform the orientation into Euler-angles ...
    [p_cl, eul_cl] = WBM.utilities.tfms.frame2posEul(vqT_cl);

    % desired pose (using vector Euler-angle transformation (veT)):
    veT_cl = vertcat(p_cl, eul_cl);
end

function [vqT_cl, quat_cl] = calcDesiredRefPoseQuat(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx)
    % calculate the desired (reference) pose for the contact link in dependency
    % of the current floating-base state:
    clnk_name = obj.mwbm_config.ccstr_link_names{1,clnk_idx};

    % get the forward kinematic transformation (clink-to-world) of the contact link ...
    vqT_cl = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, clnk_name);
    % get the quaternion part of the transformation ...
    quat_cl = vqT_cl(4:7,1);
end

function nLnks = getLinkNbr(clnk_idx)
    nLnks = size(clnk_idx,2); % row-vector
    if ( (nLnks < 1) || (nLnks > 2) )
        error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_VEC_LEN);
    end
end

function checkCState(cstate)
    if ( ~islogical(cstate) || ~isrow(cstate) )
        error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    WBM.utilities.chkfun.checkRVecDim(cstate, 2, 'WBM::clnkConfigState');
end

function checkGainValues(k_p, k_v)
    if ( ~isreal(k_p) || ~isreal(k_v) ) % no complex values are allowed ...
        error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end

function checkGainValue(k)
    if ~isreal(k)
        error('WBM::clnkConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end
