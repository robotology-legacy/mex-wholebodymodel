function checkPayloadLink(pl_link_name, func_name)
    if ~isempty(find(strcmp(pl_link_name, {'none', ''}), 1))
        error('%s: %s', func_name, WBM.wbmErrorMsg.PL_LNK_NOT_DEF);
    end
end
