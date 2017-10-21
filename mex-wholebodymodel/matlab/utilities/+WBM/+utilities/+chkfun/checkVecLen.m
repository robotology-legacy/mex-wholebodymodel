function checkVecLen(v, len, func_name)
    if (length(v) ~= len)
        error('%s: %s', func_name, WBM.wbmErrorMsg.WRONG_VEC_LEN);
    end
end
