function clink_conf = setCLinkConfigState(obj, clink_l, clink_r, varargin)
    switch nargin
        case 9
            q_j    = varargin{1,1};
            veT_ll = varargin{1,2};
            veT_rl = varargin{1,3};

            WBM.utilities.checkCVecDs(veT_ll, veT_rl, 6, 6, 'WBM::setCLinkConfigState');

            % get the link orientations ...
            eul_ll = veT_ll(4:6,1);
            eul_rl = veT_rl(4:6,1);

            contact = varargin{1,4};
            k_p     = varargin{1,5}; % gain k_p with the desired closed-loop stiffness (1)
            k_v     = varargin{1,6}; % gain k_v with the desired closed-loop damping (2)
        case {6, 7}
            q_j     = varargin{1,1};
            contact = varargin{1,2};
            k_p     = varargin{1,3}; % (1)

            if (nargin == 7)
                k_v = varargin{1,4}; % (2)
            else
                % set k_v for critical damping (3)
                % Source: Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition,
                %         Pearson/Prentice Hall, 2005, p. 274, eq. (9.47).
                k_v = 2*sqrt(k_p);
            end
        case 5
            q_j     = varargin{1,1};
            contact = varargin{1,2};

            k_p = obj.DF_STIFFNESS; % use the default stiffness value ...
            k_v = 2*sqrt(k_p); % (3)
        otherwise
            error('WBM::setCLinkConfigState: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % some error checks ...
    if ( ~islogical(contact) || ~isrow(contact) )
        error('WBM::setCLinkConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    if ( ~isreal(k_p) || ~isreal(k_v) ) % no complex values are allowed ...
        error('WBM::setCLinkConfigState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end

    %% Setup the configuration structure for the qualitative state of the contact links:
    clink_conf.lnk_l = clink_l;
    clink_conf.lnk_r = clink_r;
    % set the correction values (gains) for the links to avoid numerical integration errors:
    clink_conf.ctrl_gains.k_p = k_p; % control gain for the link positions.
    clink_conf.ctrl_gains.k_v = k_v; % control gain for the link velocities (linear & angular).

    % define on which link the robot is in contact with the ground/object:
    clink_conf.contact.left  = contact(1,1);
    clink_conf.contact.right = contact(1,2);

    % set the desired pose of the links as reference:
    if (nargin == 9)
        clink_conf.des_pose.veT_llnk = veT_ll; % left contact link
        clink_conf.des_pose.veT_rlnk = veT_rl; % right contact link
    else
        % calculate the desired (reference) pose for the contact links in dependency
        % of the current floating base state:
        % get the forward kinematic transformations of the links ...
        stFltb = getFloatingBaseState(obj);
        wf_R_b_arr = reshape(stFltb.wf_R_b, 9, 1);
        vqT_ll = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, stFltb.wf_p_b, q_j, clink_l);
        vqT_rl = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, stFltb.wf_p_b, q_j, clink_r);
        % get the positions and transform the orientations into Euler-angles ...
        [p_ll, eul_ll] = WBM.utilities.frame2posEul(vqT_ll);
        [p_rl, eul_rl] = WBM.utilities.frame2posEul(vqT_rl);

        % desired pose (using vector Euler-angle transformations (veT)):
        clink_conf.des_pose.veT_llnk = vertcat(p_ll, eul_ll);
        clink_conf.des_pose.veT_rlnk = vertcat(p_rl, eul_rl);
    end

    % set the angular velocity transformations for the contact links:
    clink_conf.des_pose.avT_llnk = WBM.utilities.eul2angVelTF(eul_ll);
    clink_conf.des_pose.avT_rlnk = WBM.utilities.eul2angVelTF(eul_rl);
end
