function checkIdxListBounds(idx_list, len, ndof, func_name)
    if ( (len > ndof) || (idx_list(1,1) < 1) || (idx_list(1,len) > ndof) )
        error('%s: %s', func_name, WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
    end
end
