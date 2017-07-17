function checkCLinkConfig(clink_conf, func_name)
    if ~isstruct(clink_conf)
        error('%s: %s', func_name, WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end
