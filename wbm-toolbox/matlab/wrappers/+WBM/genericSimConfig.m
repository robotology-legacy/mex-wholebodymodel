classdef genericSimConfig < WBM.absSimConfig
    properties(Constant)
        DF_MAIN_POS@double     vector   = [50 400 600 650];
        DF_AXES_POS@double     matrix   = [0.51 0.20 0.45 0.40;
                                           0.01 0.20 0.45 0.40;
                                           0.51 0.62 0.45 0.40;
                                           0.01 0.62 0.45 0.40];
        DF_AXES_COLORS@double  matrix   = repmat([0.8 0.8 0.8], 4, 1);
        DF_AXIS_LIMITS@double  vector   = [-0.5 0.5 -0.42 0.58 0 1];

        DF_GROUND_SHAPE@double matrix   = [-0.45 -0.45 0.45 0.45; % x
                                           -0.37 0.53 0.53 -0.37; % y
                                           0 0 0 0];              % z
        DF_GROUND_COLOR@double vector   = [0.6 0.6 0.8];
        DF_GROUND_COLOR_2@double vector = [201 220 222] ./ 255; % hex: #c9dcde
    end

    properties
        robot_body@WBM.wbmSimBody
        environment@WBM.wbmSimEnvironment

        hMainFigure@matlab.ui.Figure
        main_title@char = '';
        main_pos@double     vector = WBM.genericSimConfig.DF_MAIN_POS;

        hAxes@double        vector = zeros(1,4);
        axes_pos@double     matrix = WBM.genericSimConfig.DF_AXES_POS;
        axes_colors@double  matrix = WBM.genericSimConfig.DF_AXES_COLORS;
        axis_limits@double  vector = WBM.genericSimConfig.DF_AXIS_LIMITS;

        plot_objs@cell      vector = cell(1,4);
    end

    methods
        function obj = genericSimConfig(main_title, robot_sim_body, env_settings)
            if ( (nargin < 2) || (nargin > 3) )
                error('genericSimConfig::genericSimConfig: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            obj.main_title = main_title;
            obj.robot_body = robot_sim_body;

            if exist('env_settings', 'var')
                obj.environment = env_settings;
                return
            end
            % else, use the default environment settings ...
            obj.environment = WBM.wbmSimEnvironment;
            obj.environment.background_color_opt = 'white';
            obj.environment.ground_shape         = WBM.genericSimConfig.DF_GROUND_SHAPE;
            obj.environment.ground_color         = WBM.genericSimConfig.DF_GROUND_COLOR;
            obj.environment.ground_edge_color    = 'black';
            obj.environment.origin_pt_color      = 'black';
            obj.environment.origin_pt_size       = 3.5;
        end

    end
end
