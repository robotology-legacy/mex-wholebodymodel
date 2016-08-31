function vis_data = getFwdDynVisualizationData(obj, stmChi, fhTrqControl, foot_conf)
    extd = false;
    if exist('foot_conf', 'var')
        % use the extended forward dynamic method ...
        extd = true;
    end

    ndof   = obj.mwbm_model.ndof;
    nCstrs = obj.mwbm_config.nCstrs;
    len_fc = 6*nCstrs; % length of the contact force vector.

    [noi, stvLen] = size(stmChi);
    if (stvLen ~= obj.mwbm_config.stvLen)
        error('WBM::getFwdDynVisualizationData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end

    % read the first data-structure such that the complete dynamic data container
    % (dynamic structure) can be generated for the visualization data:
    stvChi = stmChi(1,1:stvLen).';
    if extd
        fdyn_data = getFDynDataExt(obj, 1, stvChi, fhTrqControl, foot_conf);
    else
        fdyn_data = getFDynData(obj, 1, stvChi, fhTrqControl);
    end
    % create and initialize the fields for the dynamic visualization data structure:
    [vis_data, field_names, nFields] = initVisDataStruct(fdyn_data, noi, ndof, len_fc);

    % add all data to the fields of the generated data container:
    if extd
        % extended forward dynamics:
        for i = 1:noi
            stvChi = stmChi(i,1:stvLen).';

            fdyn_data = getFDynDataExt(obj, i, stvChi, fhTrqControl, foot_conf);
            vis_data  = setVisData(vis_data, i, fdyn_data, field_names, nFields, ndof, len_fc);
        end
    else
        % normal (simple) forward dynamics:
        for i = 1:noi
            stvChi = stmChi(i,1:stvLen).';

            fdyn_data = getFDynData(obj, i, stvChi, fhTrqControl);
            vis_data  = setVisData(vis_data, i, fdyn_data, field_names, nFields, ndof, len_fc);
        end
    end
end
%% END of getFwdDynVisualizationData.


function fdyn_data = getFDynData(obj, t, stvChi, fhTrqControl)
    % get the current state parameters ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_data] = fhTrqControl(t);
    % get the the visualization data from the joint-acceleration computation:
    [~,fdyn_data] = jointAccelerations(obj, stp.dq_j, tau); % optimized mode
    % add the controller data to the data-structure:
    fdyn_data.ctrl_data = ctrl_data;
end

function fdyn_data = getFDynDataExt(obj, t, stvChi, fhTrqControl, foot_conf)
    % get the state parameters ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu  = vertcat(v_b, stp.dq_j);         % mixed generalized velocity

    % update the state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    M    = mexWholeBodyModel('mass-matrix');
    C_qv = mexWholeBodyModel('generalised-forces');
    % compute the constraint (contact) Jacobians:
    [Jc, dJcdq] = contactJacobians(obj);

    % get the torque forces and the visualization data from the controller:
    [tau, ctrl_data] = fhTrqControl(t, M, C_qv, stp, nu, Jc, dJcdq, foot_conf);
    % get the visualization data from the acceleration calculation:
    [~,fdyn_data] = jointAccelerationsExt(obj, M, C_qv, stp.dq_j, nu, tau, Jc, dJcdq, foot_conf); % optimized mode
    % add the controller data to the visualization data:
    fdyn_data.ctrl_data = ctrl_data;
end

function [vis_data, field_names, nFields] = initVisDataStruct(fdyn_data, noi, ndof, len_fc)
    % basic initialization:
    vis_data = struct('f_c', zeros(len_fc,noi), 'tau', zeros(ndof,noi), 'tau_gen', zeros(6+ndof,noi));

    % get the field-names of the dynamic controller data structure:
    field_names = fieldnames(fdyn_data.ctrl_data);
    nFields     = size(field_names,1);
    if (nFields == 0)
        % the data structure is empty ...
        return
    end

    % add the new field-names to the visualization data container and initialize them:
    for i = 1:nFields
        field_name = field_names{i,1};
        [m,n] = size(fdyn_data.ctrl_data.(field_name)); % field dimension ...

        % initialize the new data-field ...
        if iscolumn(field)
            vis_data.(field_name) = zeros(m,noi);
        elseif isrow(field)
            vis_data.(field_name) = zeros(noi,n);
        elseif ismatrix(field)
            vis_data.(field_name) = cell(noi,1);
        elseif isscalar(field)
            vis_data.(field_name) = zeros(noi,1);
        else
            error('initVisDataStruct: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end

function vis_data = setVisData(vis_data, idx, fdyn_data, field_names, nFields, ndof, len_fc)
    % add the current basic data to the data container:
    vis_data.f_c(1:len_fc,idx)       = fdyn_data.f_c;
    vis_data.tau(1:ndof,idx)         = fdyn_data.tau;
    vis_data.tau_gen(1:(ndof+6),idx) = fdyn_data.tau_gen;

    % add the current controller values to the dynamic fields:
    for i = 1:nFields
        field_name = field_names{i,1};
        field = fdyn_data.ctrl_data.(field_name);
        [m,n] = size(field); % field dimension ...

        % set the dynamic data-field ...
        if iscolumn(field)
            vis_data.(field_name)(1:m,idx) = field;
        elseif isrow(field)
            vis_data.(field_name)(idx,1:n) = field;
        elseif ismatrix(field)
            vis_data.(field_name){idx,1} = field;
        elseif isscalar(field)
            vis_data.(field_name)(idx,1) = field;
        else
            error('setVisData: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end
