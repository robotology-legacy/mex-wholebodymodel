function clink_conf = ctcLinksConfigState(obj, varargin)
    tf_data = false;

    switch nargin
        case 6
            cstate   = varargin{1,1};
            clnk_idx = varargin{1,2};
            k_p      = varargin{1,4}; % gain k_p with the desired closed-loop stiffness (1)
            k_v      = varargin{1,5}; % gain k_v with the desired closed-loop damping (2)

            ndof = obj.mwbm_model.ndof;
            v    = varargin{1,3};
            if iscolumn(v)
                if (size(v,1) == ndof)
                    q_j = v;
                else
                    WBM.utilities.chkfun.checkCVecDim(v, 6, 'WBM::ctcLinksConfigState');
                    veT_lnk = v;
                    tf_data = true;
                end
            elseif ismatrix(v)
                WBM.utilities.chkfun.checkMatDim(v, 6, 2, 'WBM::ctcLinksConfigState');
                veT_lnk = v;
                tf_data = true;
            else
                error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            checkGainValues(k_p, k_v);
        case 5
            cstate   = varargin{1,1};
            clnk_idx = varargin{1,2};
            q_j      = varargin{1,3};
            k_p      = varargin{1,4}; % (1)

            checkGainValue(k_p);
            % set k_v for critical damping (3)
            % Source: Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition,
            %         Pearson/Prentice Hall, 2005, p. 274, eq. (9.47).
            k_v = 2*sqrt(k_p);
        case 4
            cstate   = varargin{1,1};
            clnk_idx = varargin{1,2};
            q_j      = varargin{1,3};

            k_p = obj.DF_STIFFNESS; % use the default stiffness value ...
            k_v = 2*sqrt(k_p); % (3)
        case 1
            clink_conf.contact.left  = false;
            clink_conf.contact.right = false;
            return
        otherwise
            error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % further error checks ...
    if ( ~islogical(cstate) || ~isrow(cstate) )
        error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    WBM.utilities.chkfun.checkRVecDim(cstate, 2, 'WBM::ctcLinksConfigState');

    nLnks = size(clnk_idx,2); % row-vector
    if ( (nLnks < 1) || (nLnks > 2) )
        error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.WRONG_VEC_SIZE);
    end

    %% Setup the configuration structure for the
    %  qualitative state of the contact links:
    ctc_l = cstate(1,1);
    ctc_r = cstate(1,2);
    % define which link of the robot has
    % surface contact with the ground/object ...
    clink_conf.contact.left  = ctc_l;
    clink_conf.contact.right = ctc_r;

    if (nLnks == 2)
        % both contact links (in independence of
        % the contact state condition):
        clnk_idx_l = clnk_idx(1,1);
        clnk_idx_r = clnk_idx(1,2);

        clink_conf.lnk_idx_l = clnk_idx_l;
        clink_conf.lnk_idx_r = clnk_idx_r;

        % set the desired pose of the links as reference:
        if tf_data
            checkTFDim(veT_lnk, nLnks);
            % get the vector Euler-angle transformations (veT) ...
            clink_conf.des_pose.veT_llnk = veT_lnk(1:6,1); % left contact link
            clink_conf.des_pose.veT_rlnk = veT_lnk(1:6,2); % right contact link
            eul_ll = veT_lnk(4:6,1);
            eul_rl = veT_lnk(4:6,2);
        else
            % calculate the desired poses (using veT) ...
            stFltb = getFloatingBaseState(obj);
            wf_R_b_arr = reshape(stFltb.wf_R_b, 9, 1);
            [clink_conf.des_pose.veT_llnk, eul_ll] = calcDesiredRefPose(obj, wf_R_b_arr, stFltb.wf_p_b, q_j, clnk_idx_l);
            [clink_conf.des_pose.veT_rlnk, eul_rl] = calcDesiredRefPose(obj, wf_R_b_arr, stFltb.wf_p_b, q_j, clnk_idx_r);
        end
        % set the angular velocity transformations for the contact links:
        clink_conf.des_pose.avT_llnk = WBM.utilities.tfms.eul2angVelTF(eul_ll);
        clink_conf.des_pose.avT_rlnk = WBM.utilities.tfms.eul2angVelTF(eul_rl);
    elseif ( nLnks && ctc_l ) % nLnks = 1
        % only left contact link:
        clink_conf.lnk_idx_l = clnk_idx;

        % set the desired reference pose:
        if tf_data
            checkTFDim(veT_lnk, nLnks);

            clink_conf.des_pose.veT_llnk = veT_lnk;
            eul_ll = veT_lnk(4:6,1);
        else
            % calculate desired pose (veT) ...
            stFltb = getFloatingBaseState(obj);
            wf_R_b_arr = reshape(stFltb.wf_R_b, 9, 1);
            [clink_conf.des_pose.veT_llnk, eul_ll] = calcDesiredRefPose(obj, wf_R_b_arr, stFltb.wf_p_b, q_j, clnk_idx);
        end
        % angular velocity transformation (avT):
        clink_conf.des_pose.avT_llnk = WBM.utilities.tfms.eul2angVelTF(eul_ll);
    elseif ( nLnks && ctc_r ) % nLnks = 1
        % only right contact link:
        clink_conf.lnk_idx_r = clnk_idx;

        % desired reference pose:
        if tf_data
            checkTFDim(veT_lnk, nLnks);

            clink_conf.des_pose.veT_rlnk = veT_lnk;
            eul_rl = veT_lnk(4:6,1);
        else
            % calculate pose (veT) ...
            stFltb = getFloatingBaseState(obj);
            wf_R_b_arr = reshape(stFltb.wf_R_b, 9, 1);
            [clink_conf.des_pose.veT_rlnk, eul_rl] = calcDesiredRefPose(obj, wf_R_b_arr, stFltb.wf_p_b, q_j, clnk_idx);
        end
        % angular velocity transformation (avT):
        clink_conf.des_pose.avT_rlnk = WBM.utilities.tfms.eul2angVelTF(eul_rl);
    else
        % both contact links are not in contact with any surface ...
        error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.NO_LNK_IN_CTC);
    end
    % set the correction values (gains) for the links
    % to avoid numerical integration errors:
    clink_conf.ctrl_gains.k_p = k_p; % control gain for the link positions.
    clink_conf.ctrl_gains.k_v = k_v; % control gain for the link velocities (linear & angular).
end
%% END of ctcLinksConfigState.


%% DESIRED REFERENCE POSE, CHECK GAINS & VE-TRANSFORMATIONS:

function [veT_cl, eul_cl] = calcDesiredRefPose(obj, wf_R_b_arr, wf_p_b, q_j, clnk_idx)
    % calculate the desired (reference) pose for the contact link in dependency
    % of the current floating base state:
    clnk_name = obj.mwbm_config.ccstr_link_names{1,clnk_idx};

    % get the forward kinematic transformation (clink-to-world) of the contact link ...
    vqT_cl = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, clnk_name);
    % get the position and transform the orientation into Euler-angles ...
    [p_cl, eul_cl] = WBM.utilities.tfms.frame2posEul(vqT_cl);

    % desired pose (using vector Euler-angle transformation (veT)):
    veT_cl = vertcat(p_cl, eul_cl);
end

function checkGainValues(k_p, k_v)
    if ( ~isreal(k_p) || ~isreal(k_v) ) % no complex values are allowed ...
        error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end

function checkGainValue(k)
    if ~isreal(k)
        error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end

function checkTFDim(veT_lnk, nLnks)
    % check the number of VE-transformations ...
    if (size(veT_lnk,2) ~= nLnks)
        error('WBM::ctcLinksConfigState: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
    end
end
