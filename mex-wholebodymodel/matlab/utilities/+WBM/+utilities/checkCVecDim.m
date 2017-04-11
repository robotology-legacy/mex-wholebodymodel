function checkCVecDim(v, len, func_name)
    if (size(v,1) ~= len)
        error('%s: %s', func_name, WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
end
