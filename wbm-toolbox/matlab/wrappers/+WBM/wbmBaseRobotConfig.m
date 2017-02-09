classdef wbmBaseRobotConfig
    properties
        nCstrs@uint16        scalar = 0;
        cstr_link_names@cell vector
        nPlds@uint16         scalar = 0;
        payload_links@WBM.wbmPayloadLink vector = WBM.wbmPayloadLink.empty;
        nTools@uint16        scalar = 0;
        tool_links@WBM.wbmToolLink vector = WBM.wbmToolLink.empty;
        init_state_params@WBM.wbmStateParams = WBM.wbmStateParams;
        stvLen@uint16        scalar = 0;
        body@WBM.wbmBody
    end
end
