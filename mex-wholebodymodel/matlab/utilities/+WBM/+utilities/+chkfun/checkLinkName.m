function checkLinkName(lnk_name, func_name)
    if ~isempty(find(strcmp(lnk_name, {'none', ''}), 1))
        error('%s: %s', func_name, WBM.wbmErrorMsg.LNK_NOT_DEF);
    end
end
