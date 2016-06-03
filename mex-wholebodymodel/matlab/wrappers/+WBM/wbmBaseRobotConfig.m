classdef wbmBaseRobotConfig
    properties
       nCstrs@uint16        scalar
       cstr_link_names@cell vector = {};
       nPlds@uint16         scalar = 0;
       lnk_payloads@WBM.wbmLinkPayload vector
       init_state_params@WBM.wbmStateParams = WBM.wbmStateParams;
       stvLen@uint16        scalar
       body@WBM.wbmBody
    end
end
