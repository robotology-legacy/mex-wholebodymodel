classdef wbmToolLink
    properties
        urdf_link_name@char     = 'none';
        ee_vqT_tt@double vector = zeros(3,1);

        pl_idx@uint8     scalar = 0;
    end
end
