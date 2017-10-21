classdef wbmRobotModel < handle
    properties
        ndof@uint16        scalar = 0;
        urdf_robot_name@char
        urdf_fixed_link@char % reference link (= floating base link) to the world frame (WF).
        wf_R_b_init@double matrix = eye(3,3);   % initial orientation of the base frame to the WF.
        wf_p_b_init@double vector = zeros(3,1); % initial position of the base frame to the WF.
        g_wf@double        vector = zeros(3,1); % gravity in the WF.
        frict_coeff = struct( 'v', [], ...
                              'c', [] );
        jlmts       = struct( 'lwr', [], ...
                              'upr', [] );
    end
end
