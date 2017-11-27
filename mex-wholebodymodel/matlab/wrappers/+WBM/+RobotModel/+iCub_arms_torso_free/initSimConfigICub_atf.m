function sim_config = initSimConfigICub_atf(urdf_file_name, scn_mode)
    if ~exist('scn_mode', 'var')
        scn_mode = 'LightScn'; % default scene for the simulation.
    end

    % List of link and frame names that are deduced from their 'parent joints' or
    % 'parent links' of the iCub-Robot model. The only exception is the last link,
    % the 'com', which does not have a parent joint and also not a parent link.
    %
    % The link names 'r_gripper' and 'l_gripper' are aliases for the link frames
    % 'r_hand_dh_frame' and 'l_hand_dh_frame', where their origins are placed on
    % the axis of their parent joints. In general, the link names are describing
    % the "virtual joints" that are used to construct the dynamic model of the
    % iCub-Robot in the visualizer.
    %
    % Additionally each link in the list has a vector with scale factors. The scale
    % factors are defining the sizes of the shapes (patches) for the corresponding
    % links (edges) of the robot's connectivity graph. They form the shape of the
    % robot's body.
    %
    % Further details about the definitions of the joint, link and frame names are
    % available in the iCub Model Naming Conventions under,
    %    <http://wiki.icub.org/wiki/ICub_Model_naming_conventions>.
    %
    % Link names with their size vectors to define the body shape of the robot:
    %                    link             x      y
    %                                    wid    hgt
    %                    left leg:
    lnk_shape_sizes = {'l_lower_leg',  [0.035  0.03]; ...
                       'l_ankle_1',    [0.025  0.03]; ...
                       % right leg:
                       'r_lower_leg',  [0.035  0.03]; ...
                       'r_ankle_1',    [0.025  0.03]; ...
                       % prevent drawing of the dh-frames and the skin-frame:
                       'l_hand_dh_frame',     [0  0];
                       'r_hand_dh_frame',     [0  0]; ...
                       'l_forearm_dh_frame',  [0  0]; ...
                       'r_forearm_dh_frame',  [0  0]; ...
                       'l_foot_dh_frame',     [0  0]; ...
                       'r_foot_dh_frame',     [0  0]; ...
                       'chest_skin_frame',    [0  0]; ...
                       % torso:
                       'l_hip_1',       [0.02  0.02]; ...
                       'l_shoulder_1',  [0.02  0.02]; ...
                       'r_hip_1',       [0.02  0.02]; ...
                       'r_shoulder_1',  [0.02  0.02]; ...
                       % left arm:
                       'l_shoulder_2',  [0.01  0.01]; ...
                       'l_shoulder_3',  [0.01  0.01]; ...
                       'l_forearm',     [0.01  0.01]; ...
                       'l_gripper',    [0.01  0.035]; ...
                       'l_hand',        [0.01  0.01]; ...
                       % right arm:
                       'r_shoulder_2',  [0.01  0.01]; ...
                       'r_shoulder_3',  [0.01  0.01]; ...
                       'r_forearm',     [0.01  0.01]; ...
                       'r_gripper',    [0.01  0.035]; ...
                       'r_hand',        [0.01  0.01]; ...
                       % head:
                       'imu_frame',     [0.05  0.05]; ...
                       'head',          [0.01  0.01]; ...
                       'neck_2',        [0.02  0.02]};

    % This function reads out all necessary data from the given URDF-file.
    % It generates the joint-link name list, the joint-pair index list for creating
    % the connectivity graph (skeleton), the complete shape-size list for the
    % robot hull and it returns a joint-index pair for placing the feet on the
    % right positions.
    %
    sf = 0.03; % default scale factor for the shape sizes of the links.
    %sf = 0.25;
    [jnt_lnk_names,~, jnt_pair_idx,~, shape_geom.size_sf, foot_geom.joints] = WBM.utilities.getSimConfigFromURDF(urdf_file_name, lnk_shape_sizes, sf);

    % Connection matrix to define which vertices are to connect for creating the shapes
    % of the links or the shapes of the feet. Each row represents one polygon (rectangle):
    shape_geom.faces = uint8([1 2 3 4;
                              1 4 8 5;
                              2 6 5 1;
                              3 7 8 4;
                              5 8 7 6;
                              7 3 2 6]);

    % Base size values for the feet of the robot:
    foot_geom.base_sz.width  = 0.03; %0.025;
    foot_geom.base_sz.height = 0.015;

    % List with additional size values to define the foot dimensions:
    %                       x:     y:  z:
    %                      len    wid hgt
    foot_geom.shape_ds = [-0.02    0   0; % up back
                          -0.02    0   0;
                          -0.035   0   0; % bottom back (back length)
                          -0.035   0   0;
                           0.03    0   0; % up front
                           0.03    0   0;
                           0.12    0   0; % bottom front (front length)
                           0.12    0   0];

    % Create the body model of the animated robot in the simulation:
    sim_body = WBM.wbmSimBody(jnt_lnk_names, jnt_pair_idx);

    % Geometry properties for the shape of the robot body:
    sim_body.shape_geom = shape_geom;
    sim_body.foot_geom  = foot_geom;

    % Draw properties for the robot body and environment settings for the animated scene:
    sim_body.draw_prop.joints.color = WBM.wbmColor.orange;

    env_settings = WBM.wbmSimEnvironment;
    env_settings.grnd_shape   = WBM.genericSimConfig.DF_GROUND_SHAPE;
    env_settings.orig_pt_size = 4.5;

    % Set scene mode:
    switch scn_mode
        case 'LightScn'
            % draw settings for the light scene ...
            sim_body.draw_prop.links.color      = 'black';
            sim_body.draw_prop.shape.face_color = WBM.wbmColor.royalblue4;
            sim_body.draw_prop.shape.edge_color = WBM.wbmColor.royalblue4;

            env_settings.bkgrd_color_opt = 'white';
            env_settings.grnd_color      = WBM.wbmColor.papayawhip;
            env_settings.grnd_edge_color = 'black';
            env_settings.orig_pt_color   = 'black';
        case 'DarkScn'
            % draw settings for the dark scene ...
            sim_body.draw_prop.links.color      = WBM.wbmColor.seashell4;
            sim_body.draw_prop.shape.edge_color = WBM.wbmColor.steelblue;
            sim_body.draw_prop.shape.face_color = WBM.wbmColor.steelblue;

            env_settings.bkgrd_color_opt = 'black';
            env_settings.grnd_color      = WBM.wbmColor.snow;
            env_settings.grnd_edge_color = 'none';
            env_settings.orig_pt_color   = WBM.wbmColor.violetred;
        otherwise
            error('initSimConfigICub: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end

    % Create the configuration object for the WBM-Simulator:
    sim_config = WBM.genericSimConfig('iCub-Simulator:', sim_body, 2, env_settings);
    % set new window and axes settings ...
    sim_config.wnd_pos    = [60  600];
    sim_config.wnd_size   = [600 300];
    sim_config.axes_views = {'custom', 'side_l'};
    sim_config.axes_pos   = [0.54  0.09  0.45  0.90;
                             0.01  0.09  0.45  0.90];
end
