classdef wbmBaseModelParams
    properties
       urdfRobot@char
       urdfLinkName@char
       wf_R_rootLnk = zeros(3,3);
       wf_p_rootLnk = zeros(3,1);
       g_wf         = zeros(3,1);
    end
end