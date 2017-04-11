function checkRVecDim(v, len, func_name)
    if (size(v,2) ~= len)
        error('%s: %s', func_name, WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
end
