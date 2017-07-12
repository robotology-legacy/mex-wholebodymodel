function checkValLTZero(val, func_name)
    if (val < 0)
        error('%s: %s', func_name, WBM.wbmErrorMsg.VALUE_LT_ZERO);
    end
end
