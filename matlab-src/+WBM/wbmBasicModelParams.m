classdef wbmBasicModelParams
    properties
       urdfRobot@char
       urdfRefLinkName@char
       wf_R_rootLnk = zeros(3);
       wf_p_rootLnk = zeros(3,1);
       %wf_R_refLnk  = zeros(3);
       %wf_p_refLnk  = zeros(3,1);
       g_wf         = zeros(3,1);
    end
end