classdef wbmPayloadLink
    properties
        urdf_link_name@char
        lnk_p_cm@double     vector = zeros(3,1);

        vb_idx@uint16       scalar = 0;
        vb_grabbed@logical  scalar = false;
        vb_released@logical scalar = false;

        m_rb@double         scalar = 0;
        I_cm@double         matrix = zeros(3,3);
    end
end
