function vis_data = getFwdDynVisualizationData(obj, stmChi, fhTrqControl, clink_conf)
    extd = false;
    if exist('clink_conf', 'var')
        % use the extended forward dynamic method ...
        extd = true;
    end

    ndof   = obj.mwbm_model.ndof;
    nCstrs = obj.mwbm_config.nCstrs;
    vlen = 6*nCstrs; % vector length of f_c, f_e & a_c.

    [noi, stvLen] = size(stmChi);
    if (stvLen ~= obj.mwbm_config.stvLen)
        error('WBM::getFwdDynVisualizationData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end

    % read the first data-structure such that the complete dynamic data container
    % (dynamic structure) can be generated for the visualization data:
    stvChi = stmChi(1,1:stvLen).';
    if extd
        fd_prms = getFDynParamsCLPC(obj, 1, stvChi, fhTrqControl, clink_conf);
    else
        fd_prms = getFDynParams(obj, 1, stvChi, fhTrqControl);
    end
    % create and initialize the fields for the dynamic visualization data structure:
    [vis_data, nflds_base, nflds_ctrl, fnames_ctrl] = initVisDataStruct(fd_prms, noi, ndof, vlen);

    % add all data to the fields of the generated data container:
    if extd
        % extended forward dynamics:
        for i = 1:noi
            stvChi = stmChi(i,1:stvLen).';

            fd_prms  = getFDynParamsCLPC(obj, i, stvChi, fhTrqControl, clink_conf);
            vis_data = setVisData(vis_data, i, ndof, vlen, fd_prms, nflds_base, nflds_ctrl, fnames_ctrl);
        end
    else
        % normal (simple) forward dynamics:
        for i = 1:noi
            stvChi = stmChi(i,1:stvLen).';

            fd_prms  = getFDynParams(obj, i, stvChi, fhTrqControl);
            vis_data = setVisData(vis_data, i, ndof, vlen, fd_prms, nflds_base, nflds_ctrl, fnames_ctrl);
        end
    end
end
%% END of getFwdDynVisualizationData.


function fd_prms = getFDynParams(obj, t, stvChi, fhTrqControl)
    % get the current state parameters ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t);
    % get the the visualization data from the joint-acceleration computation:
    [~,fd_prms] = jointAccelerations(obj, tau, stp.dq_j); % optimized mode
    % add the controller data to the data-structure:
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsCLPC(obj, t, stvChi, fhTrqControl, clink_conf)
    % get the state parameters ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu  = vertcat(v_b, stp.dq_j);         % mixed generalized velocity

    % update the state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    M    = mexWholeBodyModel('mass-matrix');
    c_qv = mexWholeBodyModel('generalized-forces');
    % compute the constraint (contact) Jacobians:
    [Jc, djcdq] = contactJacobians(obj);

    % get the torque forces and the visualization data from the controller:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu, Jc, djcdq, clink_conf);
    % get the visualization data from the acceleration calculation (optimized mode):
    f_e = obj.ZERO_EX_FORCE;
    a_c = obj.ZERO_CTC_ACC;
    [~,fd_prms] = jointAccelerationsCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, M, c_qv, stp.dq_j, nu);

    % add the controller data to the visualization data:
    fd_prms.ctrl_prms = ctrl_prms;
end

function [vis_data, nflds_base, nflds_ctrl, fnames_ctrl] = initVisDataStruct(fd_prms, noi, ndof, vlen)
    % basic initialization:
    fnames = fieldnames(fd_prms);
    nflds_base = size(fnames,1);
    if (nflds_base > 2)
        vis_data = struct('tau_gen', zeros(ndof+6,noi), 'f_c', zeros(vlen,noi), ...
                          'f_e', zeros(vlen,noi), 'a_c', zeros(vlen,noi));
    else
        vis_data = struct('tau_gen', zeros(ndof+6,noi), 'f_c', zeros(vlen,noi));
    end

    % get the field-names of the dynamic controller data structure:
    fnames_ctrl = fieldnames(fd_prms.ctrl_prms);
    nflds_ctrl  = size(fnames_ctrl,1);
    if (nflds_ctrl == 0)
        % the data structure is empty ...
        return
    end

    % add the new field-names to the dynamic visualization data-field
    % and initialize them:
    for i = 1:nflds_ctrl
        fname = fnames_ctrl{i,1};
        fld   = fd_prms.ctrl_prms.(fname);
        [m,n] = size(fld); % field dimension

        if iscolumn(fld)
            vis_data.(fname) = zeros(m,noi);
        elseif isrow(fld)
            vis_data.(fname) = zeros(noi,n);
        elseif ismatrix(fld)
            vis_data.(fname) = cell(noi,1);
        elseif isscalar(fld)
            vis_data.(fname) = zeros(noi,1);
        else
            error('initVisDataStruct: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end

function vis_data = setVisData(vis_data, idx, ndof, vlen, fd_prms, nflds_base, nflds_ctrl, fnames_ctrl)
    % add the basic data to the dynamic data-field:
    vis_data.tau_gen(1:(ndof+6),idx) = fd_prms.tau_gen;
    vis_data.f_c(1:vlen,idx)         = fd_prms.f_c;

    if (nflds_base > 2)
        vis_data.f_e(1:vlen,idx) = fd_prms.f_e;
        vis_data.a_c(1:vlen,idx) = fd_prms.a_c;
    end

    % add the controller values to the dynamic data-field:
    for i = 1:nflds_ctrl
        fname = fnames_ctrl{i,1};
        fld   = fd_prms.ctrl_prms.(fname);
        [m,n] = size(fld);

        if iscolumn(fld)
            vis_data.(fname)(1:m,idx) = fld;
        elseif isrow(fld)
            vis_data.(fname)(idx,1:n) = fld;
        elseif ismatrix(fld)
            vis_data.(fname){idx,1} = fld;
        elseif isscalar(fld)
            vis_data.(fname)(idx,1) = fld;
        else
            error('setVisData: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end
