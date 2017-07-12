function checkNumListAscOrder(num_list, func_name)
    if ~issorted(num_list)
        error('%s: %s', func_name, WBM.wbmErrorMsg.LIST_NOT_SORTED);
    end
end
