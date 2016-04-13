function wbm_icub = initRobot_iCub()
    %% Base model parameters:
    icub_model = WBM.wbmBaseModelParams;
    icub_model.urdfRobot    = 'icubGazeboSim';
    %icub_model.urdfLinkName = 'l_sole';
    icub_model.wf_R_rootLnk = eye(3,3);
    icub_model.wf_p_rootLnk = zeros(3,1);
    icub_model.g_wf         = [0; 0; -9.81];

    %% Base robot config:
    icub_config = WBM.wbmHumanoidConfig;
    icub_config.ndof          = 25;
    icub_config.nCstrs        = 2;
    icub_config.cstrLinkNames = {'l_sole', 'r_sole'};
    icub_config.dampCoeff     = 0.00; %0.75;
    % Setup the body of the iCub-Robot with the initial body (joint) positions (in degrees):
    % Note: This configuration assumes an iCub-Robot with 25 DoFs.
    icub_config.body           = WBM.Robot.iCub.setupBody_iCub();
    icub_config.jpos_torso     = [-10.0; 0.0; 0.0];
    icub_config.jpos_left_arm  = [-19.7; 29.7; 0.0; 44.9; 0.0];
    icub_config.jpos_left_leg  = [25.5; 0.1; 0.0; -38.5; -5.5; -0.1];
    icub_config.jpos_right_arm = icub_config.jpos_left_arm;
    icub_config.jpos_right_leg = icub_config.jpos_left_leg;
    % Init-State Parameters:
    icub_config.initStateParams.x_b     = zeros(3,1);
    icub_config.initStateParams.qt_b    = zeros(4,1);
    icub_config.initStateParams.q_j     = [icub_config.jpos_torso; icub_config.jpos_left_arm; icub_config.jpos_right_arm; ...
                                           icub_config.jpos_left_leg; icub_config.jpos_right_leg] * (pi/180.0); % in radians
    icub_config.initStateParams.dx_b    = zeros(3,1);
    icub_config.initStateParams.omega_b = zeros(3,1);
    icub_config.initStateParams.dq_j    = zeros(icub_config.ndof,1);

    %% Initialize the mex-WholeBodyModel for the iCub-Robot:
    wf2FixLnk = true;
    wbm_icub = WBM.WBM(icub_model, icub_config, wf2FixLnk);
end
