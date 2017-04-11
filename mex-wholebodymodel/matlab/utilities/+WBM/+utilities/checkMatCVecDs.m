function checkMatCVecDs(A, v, len1, len2, func_name)
    [m, n] = size(A);
    if ( (m ~= len1) || (n ~= len2) || (size(v,1) ~= len2) )
        error('%s: %s', func_name, WBM.wbmErrorMsg.DIM_MISMATCH);
    end
end
