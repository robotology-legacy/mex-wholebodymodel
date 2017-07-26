function vis_data = getFDynVisData(obj, stmChi, fhTrqControl, varargin)
    [noi, len] = size(stmChi);
    if (len ~= obj.mwbm_config.stvLen)
        error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end

    % add all data to the fields of the generated data container:
    switch nargin
        case 8
            % extended forward dynamics with feet and hand pose correction and payload:
            fhTotCWrench = varargin{1,1};
            feet_conf    = varargin{1,2};
            hand_conf    = varargin{1,3};
            f_cp         = varargin{1,4}; % applied forces at contact points pc_i
            ac_f         = varargin{1,5}; % mixed generalized accelerations of the foot contact points

            vis_data = getVisDataFHPCPL(obj, stmChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, f_cp, ac_f, noi);
        case 7
            % extended forward dynamics with feet and hand pose correction:
            feet_conf = varargin{1,1};
            hand_conf = varargin{1,2};
            fe_h      = varargin{1,3}; % external forces (hands)
            ac_f      = varargin{1,4}; % mixed generalized accelerations of the foot contact points

            vis_data = getVisDataFHPC(obj, stmChi, fhTrqControl, feet_conf, hand_conf, fe_h, ac_f, noi);
        case 6
            % extended forward dynamics with pose correction:
            clink_conf = varargin{1,1};
            f_e        = varargin{1,2}; % external forces
            a_c        = varargin{1,3}; % mixed generalized accelerations at contact points pc_i

            vis_data = getVisDataCLPC(obj, stmChi, fhTrqControl, clink_conf, f_e, a_c, noi);
        case 3
            % normal (simple) forward dynamics without pose correction:
            % read the first data structure ...
            stvChi  = stmChi(1,1:len).';
            fd_prms = getFDynParams(obj, 1, stvChi, fhTrqControl);
            % create and initialize the visualization data container ...
            [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

            % put all data into the data container:
            for i = 1:noi
                stvChi = stmChi(i,1:len).';

                fd_prms  = getFDynParams(obj, i, stvChi, fhTrqControl);
                vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
            end
        otherwise
            error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
%% END of getFDynVisData.


%% INITIALIZATION AND GET & SET DATA FUNCTIONS:

function [v_1, vlen, is_const] = getFirstVector(vlist, noi)
    % check if the given vector list has only a single vector or
    % a time list of row vectors (matrix):
    is_const = false;
    if iscolumn(vlist)
        % vector does not change in time ...
        v_1  = vlist;
        vlen = size(vlist,1);
        is_const = true;
    elseif ismatrix(vlist)
        % get the first vector ...
        [m,n] = size(vlist);
        if (m ~= noi)
            error('getFirstVector: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
        end
        v_1  = vlist(1,1:n).';
        vlen = n;
    else
        error('getFirstVector: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end

function fd_prms = getFDynParams(obj, t, stvChi, fhTrqControl)
    % get the current state parameters ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t);
    % get the the fdyn-parameters from the joint acceleration computation:
    [~,fd_prms] = jointAccelerations(obj, tau, stp.dq_j); % optimized mode

    % add the controller data to the data structure ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsCLPC(obj, t, stvChi, fhTrqControl, clink_conf, f_e, a_c)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_lnk, djcdq_lnk] = wholeBodyDynamicsCS(obj, clink_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_lnk, djcdq_lnk, clink_conf);
    % get the fdyn-parameters from the acceleration computation (optimized mode):
    [~,fd_prms] = jointAccelerationsCLPC(obj, clink_conf, tau, f_e, a_c, Jc_lnk, djcdq_lnk, ...
                                         M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsFHPC(obj, t, stvChi, fhTrqControl, feet_conf, hand_conf, fe_h, ac_f)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, feet_conf);
    % get the fdyn-parameters of the joint accelerations (optimized mode):
    [~,fd_prms] = jointAccelerationsFHPC(obj, feet_conf, hand_conf, tau, fe_h, ac_f, ...
                                         Jc_f, djcdq_f, M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsFHPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, f_cp, ac_f)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, feet_conf);
    % get the fdyn-parameters of the joint accelerations (optimized mode):
    [~,fd_prms] = jointAccelerationsFHPCPL(obj, feet_conf, hand_conf, tau, fhTotCWrench, f_cp, ...
                                           ac_f, Jc_f, djcdq_f, M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi)
    % Basic fields:
    fnames = fieldnames(fd_prms); % get the field names ...
    nflds  = size(fnames,1);
    if isempty(find(strcmp(fnames, 'ctrl_prms'), 1))
        error('initVisDataStruct: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
    end
    % initialize all data fields for the (dynamic) visualization data structure:
    vis_data = struct();
    vis_data = initVisDataFields(vis_data, fd_prms, fnames, nflds, noi);

    % remove "ctrl_prms" from the list (is always the last element):
    nflds_base  = nflds - 1;
    fnames_base = fnames(1:nflds_base,1);

    % Controller fields:
    % get the field names of the controller data structure ...
    fnames_ctrl = fieldnames(fd_prms.ctrl_prms);
    nflds_ctrl  = size(fnames_ctrl,1);
    if (nflds_ctrl == 0)
        % the data structure is empty ...
        return
    end
    % add all data fields of the controller structure to the
    % dynamic visualization data field and initialize them:
    vis_data.ctrl_prms = initVisDataFields(vis_data.ctrl_prms, fd_prms.ctrl_prms, ...
                                           fnames_ctrl, nflds_ctrl, noi);
end

function data_flds = initVisDataFields(data_flds, param_flds, fnames, nflds, noi)
    for i = 1:nflds
        fname = fnames{i,1};
        fld   = param_flds.(fname);
        [m,n] = size(fld); % field dimension

        if isstruct(fld)
            data_flds.(fname) = struct();
        elseif isscalar(fld)
            data_flds.(fname) = zeros(noi,1);
        elseif iscolumn(fld)
            data_flds.(fname) = zeros(m,noi);
        elseif isrow(fld)
            data_flds.(fname) = zeros(noi,n);
        elseif ismatrix(fld)
            data_flds.(fname) = cell(noi,1);
        else
            error('initVisDataFields: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end

function vis_data = setVisData(idx, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl)
    % add all base data to the corresponding base fields (except to the ctrl_prms field):
    vis_data = addData2Fields(idx, vis_data, fd_prms, fnames_base, nflds_base);

    % add all controller data to the controller sub-field of the visualization data:
    vis_data.ctrl_prms = addData2Fields(idx, vis_data.ctrl_prms, fd_prms.ctrl_prms, ...
                                        fnames_ctrl, nflds_ctrl);
end

function data_flds = addData2Fields(idx, data_flds, param_flds, fnames, nflds)
    % add the data to the specified data fields:
    for i = 1:nflds
        fname = fnames{i,1};
        fld   = param_flds.(fname);
        [m,n] = size(fld);

        if isstruct(fld)
            data_flds.(fname) = fld;
        elseif isscalar(fld)
            data_flds.(fname)(idx,1) = fld;
        elseif iscolumn(fld)
            data_flds.(fname)(1:m,idx) = fld;
        elseif isrow(fld)
            data_flds.(fname)(idx,1:n) = fld;
        elseif ismatrix(fld)
            data_flds.(fname){idx,1} = fld;
        else
            error('addData2Fields: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end

function vis_data = getVisDataCLPC(obj, stmChi, fhTrqControl, clink_conf, f_e, a_c, noi)
    len = obj.mwbm_config.stvLen;

    % read the first data-structure s.t. the complete dynamic structure can be
    % generated for the visualization data:
    [fe_1, fe_len, fe_const] = getFirstVector(f_e, noi);
    [ac_1, ac_len, ac_const] = getFirstVector(a_c, noi);
    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsCLPC(obj, 1, stvChi, fhTrqControl, clink_conf, fe_1, ac_1);
    % create and initialize the fields for the visualization data container ...
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fe_const && ac_const)
        % ext. forces and contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';

            fd_prms  = getFDynParamsCLPC(obj, i, stvChi, fhTrqControl, clink_conf, fe_1, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fe_const
        % only the force vector is constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            ac     = a_c(i,1:ac_len).';

            fd_prms  = getFDynParamsCLPC(obj, i, stvChi, fhTrqControl, clink_conf, fe_1, ac);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif ac_const
        % only the acceleration vector is constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = f_e(i,1:fe_len).';

            fd_prms  = getFDynParamsCLPC(obj, i, stvChi, fhTrqControl, clink_conf, fe, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = f_e(i,1:fe_len).';
            ac     = a_c(i,1:ac_len).';

            fd_prms  = getFDynParamsCLPC(obj, i, stvChi, fhTrqControl, clink_conf, fe, ac);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataFHPC(obj, stmChi, fhTrqControl, feet_conf, hand_conf, fe_h, ac_f, noi)
    len = obj.mwbm_config.stvLen;

    % read the first data-structure s.t. the complete dynamic structure can be
    % generated for the visualization data:
    [feh_1, feh_len, feh_const] = getFirstVector(fe_h, noi);
    [acf_1, acf_len, acf_const] = getFirstVector(ac_f, noi);
    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsFHPCPL(obj, 1, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, feh_1, acf_1);
    % create and initialize the fields for the visualization data container ...
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (feh_const && acf_const)
        % ext. forces (hands) and feet contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';

            fd_prms  = getFDynParamsFHPC(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, feh_1, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif feh_const
        % only the ext. forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFHPC(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, feh_1, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif acf_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            feh    = fe_h(i,1:feh_len).';

            fd_prms  = getFDynParamsFHPC(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, feh, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            feh    = fe_h(i,1:feh_len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFHPC(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, feh, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataFHPCPL(obj, stmChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, f_cp, ac_f, noi)
    len = obj.mwbm_config.stvLen;

    % read the first data-structure s.t. the complete dynamic structure can be
    % generated for the visualization data:
    [fcp_1, fcp_len, fcp_const] = getFirstVector(f_cp, noi);
    [acf_1, acf_len, acf_const] = getFirstVector(ac_f, noi);
    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsFHPCPL(obj, 1, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, fcp_1, acf_1);
    % create and initialize the fields for the visualization data container ...
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fcp_const && acf_const)
        % payload forces and feet contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, fcp_1, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fcp_const
        % only the payload forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, fcp_1, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif acf_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, fcp, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, fcp, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end
