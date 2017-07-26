function checkRVecDs(v1, v2, len1, len2, func_name)
    if ( (size(v1,2) ~= len1) || (size(v2,2) ~= len2) )
        error('%s: %s', func_name, WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
end
