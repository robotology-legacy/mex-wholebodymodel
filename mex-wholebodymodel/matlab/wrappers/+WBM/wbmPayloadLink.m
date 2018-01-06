classdef wbmPayloadLink
    properties
        urdf_link_name@char        = 'none';
        lnk_p_cm@double     vector = zeros(3,1); % position from the frame {cm} at CoM of the object to the contact link frame {lnk}

        t_idx@uint8         scalar = 0;

        vb_idx@uint8        scalar = 0;
        vb_grabbed@logical  scalar = false;
        vb_released@logical scalar = false;

        m_rb@double         scalar = 0;
        I_cm@double         matrix = [];
    end
end
