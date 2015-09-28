classdef wbmBasicModelParams
    properties
       urdfRobotName@char
       urdfRefLinkName@char
       R_rootlnk_wf = zeros(3);
       p_rootlnk_wf = zeros(3,1);
       R_reflnk_wf  = zeros(3);
       p_reflnk_wf  = zeros(3,1);
       g_wf         = zeros(3,1);
    end
end