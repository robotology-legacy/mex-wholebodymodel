classdef wbmSimEnvironment < handle
    properties
        bkgrd_color_opt@char
        grnd_shape@double       matrix
        grnd_color
        grnd_edge_color
        orig_pt_color
        orig_pt_size@double     scalar

        vb_objects@WBM.vbObject vector = WBM.vbObject.empty;
    end
end
