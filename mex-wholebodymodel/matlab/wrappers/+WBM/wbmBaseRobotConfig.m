classdef wbmBaseRobotConfig
    properties
       nCstrs@uint16        scalar = 0;
       cstr_link_names@cell vector
       nPlds@uint16         scalar = 0;
       lnk_payloads@WBM.wbmLinkPayload vector = WBM.wbmLinkPayload.empty;
       init_state_params@WBM.wbmStateParams = WBM.wbmStateParams;
       stvLen@uint16        scalar = 0;
       body@WBM.wbmBody
    end
end
