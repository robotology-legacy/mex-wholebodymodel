function [jnt_pos_values, len] = getInitValuesFromJointNames(jnt_names, jnt_names_full, qj_init_full)
    len_full = size(jnt_names_full,1);
    len      = size(jnt_names,1);

    if ~iscolumn(jnt_names)
        error('getInitValuesFromJointNames: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    if (len > len_full)
        error('getInitValuesFromJointNames: The number of joint names exceeds the full name list!');
    end
    if (len_full ~= size(qj_init_full,1))
        % the data arrays have not the same length ...
        error('getInitValuesFromJointNames: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
    end

    % extract the wanted joint position values from the complete list:
    % note: the order of the values must have the same order as the name list.
    jnt_pos_values = zeros(len,1);
    for i = 1:len
        idx = find(strcmp(jnt_names_full, jnt_names{i,1}), 1);
        jnt_pos_values(i,1) = qj_init_full(idx,1);
    end
end
