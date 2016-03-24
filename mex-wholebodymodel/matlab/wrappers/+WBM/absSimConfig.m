classdef (Abstract) absSimConfig
    properties(Abstract)
        %robot_body@WBM.wbmSimBody
        main_title@char
        hFigure_main
        hAxes@double    vector
        plot_objs@cell  vector
    end
    
    properties(Abstract, Constant)
        MAIN_POS@double    vector
        AXES_POS@double    matrix
        AXES_COLORS@double matrix
        AXIS_LIMITS@double vector
        PATCH_SHAPE@double matrix
        PATCH_COLOR@double vector
    end

    % methods(Abstract)
    %     obj = absSimConfig(main_title, robot_joint_names)
    % end
end
