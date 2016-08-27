classdef wbmBaseRobotModel
    properties
        ndof@uint16   scalar = 0;
        urdf_robot_name@char
        urdf_fixed_link@char
        wf_R_b@double matrix = eye(3,3);
        wf_p_b@double vector = zeros(3,1);
        g_wf@double   vector = zeros(3,1);
        frict_coeff = struct( 'v', [], ...
                              'c', [] );
        jlim = struct( 'lwr', [], ...
                       'upr', [] );
    end
end
