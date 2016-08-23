function wbm_icub = initRobot_iCub(wf2fixLnk)
    %% Base (default) model parameters:
    icub_model = WBM.wbmBaseRobotModel;
    icub_model.ndof         = 25;
    icub_model.urdf_robot   = 'icubGazeboSim'; % default robot model.
    %icub_model.urdfLinkName = 'l_sole';
    icub_model.wf_R_b       = eye(3,3);
    icub_model.wf_p_b       = zeros(3,1);
    icub_model.g_wf         = [0; 0; -9.81];
    %icub_model.frict_coeff.v = repmat(0.75, icub_model.ndof, 1); % optional
    %icub_model.frict_coeff.c = repmat(0.1, icub_model.ndof, 1);

    %% Base robot config:
    icub_config = WBM.wbmHumanoidConfig;
    %icub_config.nCstrs          = 2;
    icub_config.cstr_link_names = {'l_sole', 'r_sole'};
    %icub_config.nPlds = 0;
    %icub_config.payload_links = [];

    % Setup the body of the iCub-Robot with the initial body (joint) positions (in degrees):
    % Note: This configuration assumes an iCub-Robot with 25 DoFs.
    icub_config.body           = WBM.Robot.iCub.setupBody_iCub();
    icub_config.jpos_torso     = [-10.0; 0.0; 0.0];
    icub_config.jpos_left_arm  = [-19.7; 29.7; 0.0; 44.9; 0.0];
    icub_config.jpos_left_leg  = [25.5; 0.1; 0.0; -38.5; -5.5; -0.1];
    icub_config.jpos_right_arm = icub_config.jpos_left_arm;
    icub_config.jpos_right_leg = icub_config.jpos_left_leg;
    % Init-State Parameters:
    icub_config.init_state_params.x_b     = zeros(3,1);
    icub_config.init_state_params.qt_b    = zeros(4,1);
    icub_config.init_state_params.q_j     = [icub_config.jpos_torso; icub_config.jpos_left_arm; icub_config.jpos_right_arm; ...
                                             icub_config.jpos_left_leg; icub_config.jpos_right_leg] * (pi/180.0); % in radians
    icub_config.init_state_params.dx_b    = zeros(3,1);
    icub_config.init_state_params.omega_b = zeros(3,1);
    icub_config.init_state_params.dq_j    = zeros(icub_model.ndof,1);

    %% Initialize the mex-WholeBodyModel for the iCub-Robot:
    if ~exist('wf2fixLnk', 'var')
        % by default: the world frame will be NOT set to a fixed link ...
        wbm_icub = WBM.WBM(icub_model, icub_config);
        return
    end
    % else, user specific ...
    wbm_icub = WBM.WBM(icub_model, icub_config, wf2fixLnk);
end
