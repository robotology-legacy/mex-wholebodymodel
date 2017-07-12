function [dx_b, omega_b] = baseVel2params(v_b)
    if iscolumn(v_b)
        WBM.utilities.chkfun.checkCVecDim(v_b, 6, 'baseVel2params');

        dx_b    = v_b(1:3,1);
        omega_b = v_b(4:6,1);
        return
    elseif ismatrix(v_b)
        [m, n] = size(v_b);
        if (n ~= 6)
            error('baseVel2params: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
        end

        dx_b    = v_b(1:m,1:3);
        omega_b = v_b(1:m,4:6);
        return
    end
    % else ...
    error('baseVel2params: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
end
