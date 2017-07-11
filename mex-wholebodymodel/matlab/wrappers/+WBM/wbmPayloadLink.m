classdef wbmPayloadLink
    properties
        urdf_link_name@char
        lnk_p_cm@double vector = zeros(3,1);
        m_rb@double     scalar = 0;
        I_cm@double     matrix = zeros(3,3);
    end
end
