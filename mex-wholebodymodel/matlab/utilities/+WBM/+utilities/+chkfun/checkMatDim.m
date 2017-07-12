function checkMatDim(A, len1, len2, func_name)
    [m, n] = size(A);
    if ( (m ~= len1) || (n ~= len2) )
        error('%s: %s', func_name, WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end
end
