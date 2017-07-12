function vqT_b = params2baseFrm(stParams)
    if ( isempty(stParams.x_b) || isempty(stParams.qt_b) )
        error('params2baseFrm: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
    end

    if iscolumn(stParams.x_b)
        vqT_b = vertcat(stParams.x_b, stParams.qt_b);
        return
    elseif ismatrix(stParams.x_b)
        vqT_b = horzcat(stParams.x_b, stParams.qt_b);
        return
    end
    % else ...
    error('params2baseFrm: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
end
