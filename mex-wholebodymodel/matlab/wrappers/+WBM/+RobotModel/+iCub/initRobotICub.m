function wbm_icub = initRobotICub(wf2fixlnk)
    %% Initialize the default model ('icubGazeboSim') of the iCub robot with 25 degrees of freedom:
    %
    %  Source for the joint configurations,
    %       [1] Yarp-WholeBodyInterface: <https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface.ini>

    %% Base (default) model parameters:
    icub_model = WBM.wbmRobotModel;
    icub_model.ndof            = 25;
    icub_model.urdf_robot_name = 'icubGazeboSim'; % default robot model.
    %icub_model.urdf_fixed_link = 'l_sole';        % (default) optional
    icub_model.wf_R_b_init     = eye(3,3);
    icub_model.wf_p_b_init     = zeros(3,1);
    icub_model.g_wf            = [0; 0; -9.81];

    % Friction coefficients (optional):
    %icub_model.frict_coeff.v = repmat(0.75, icub_model.ndof, 1); % viscous friction
    %icub_model.frict_coeff.c = repmat(0.1, icub_model.ndof, 1);  % Coulomb friction

    %% Base robot config:
    icub_config = WBM.wbmHumanoidConfig;
    %icub_config.nCstrs           = 2; % optional
    icub_config.ccstr_link_names = {'l_sole', 'r_sole'};
    %icub_config.nPlds = 0;
    %icub_config.payload_links = WBM.wbmPayloadLink.empty;

    % Setup the body of the iCub robot with the initial body (joint) positions (in degrees):
    % Note: This configuration assumes an iCub robot with 25 DoFs.
    icub_config.body           = WBM.RobotModel.iCub.setupBodyICub();
    icub_config.jpos_torso     = [-10.0; 0.0; 0.0];
    icub_config.jpos_left_arm  = [-19.7; 29.7; 0.0; 44.9; 0.0];
    icub_config.jpos_left_leg  = [25.5; 0.1; 0.0; -38.5; -5.5; -0.1];
    icub_config.jpos_right_arm = icub_config.jpos_left_arm;
    icub_config.jpos_right_leg = icub_config.jpos_left_leg;

    % Concatenate all joint-positions of the body:
    % Note: The order of the joints must be set in the same order as defined in the listing of ROBOT_MEX_WBI_TOOLBOX in [1].
    qj_init_body = vertcat(icub_config.jpos_torso, icub_config.jpos_left_arm, icub_config.jpos_right_arm, ...
                           icub_config.jpos_left_leg, icub_config.jpos_right_leg);

    % Init-State Parameters:
    icub_config.init_state_params.x_b     = zeros(3,1);
    icub_config.init_state_params.qt_b    = zeros(4,1);
    icub_config.init_state_params.q_j     = qj_init_body * (pi/180.0); % in radians
    icub_config.init_state_params.dx_b    = zeros(3,1);
    icub_config.init_state_params.omega_b = zeros(3,1);
    icub_config.init_state_params.dq_j    = zeros(icub_model.ndof,1);

    %% Initialize the mex-WholeBodyModel for the iCub robot:
    if ~exist('wf2fixlnk', 'var')
        % by default: the world frame will NOT be set to a fixed link ...
        wbm_icub = WBM.WBM(icub_model, icub_config);
        return
    end
    % else, user specific ...
    wbm_icub = WBM.WBM(icub_model, icub_config, wf2fixlnk);
end
