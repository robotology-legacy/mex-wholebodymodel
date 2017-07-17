function checkCLinkConfigs(clink_conf1, clink_conf2, func_name)
    if ( ~isstruct(clink_conf1) || ~isstruct(clink_conf2) )
        error('%s: %s', func_name, WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end
