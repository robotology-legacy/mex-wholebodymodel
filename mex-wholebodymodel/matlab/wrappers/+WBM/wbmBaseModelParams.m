classdef wbmBaseModelParams
    properties
       urdfRobot@char
       urdfLinkName@char
       wf_R_rootLnk@double matrix = zeros(3,3);
       wf_p_rootLnk@double vector = zeros(3,1);
       g_wf@double         vector = zeros(3,1);
    end
end
