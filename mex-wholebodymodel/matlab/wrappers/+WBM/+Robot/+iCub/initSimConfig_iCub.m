function sim_config = initSimConfig_iCub()
    % List of link and frame names that are deduced from their 'parent joints' of the
    % iCub-Robot model with the exception of the first and the last link, the 'root_link'
    % and the 'com', they don't have any parent joints.
    % The link names 'r_gripper' and 'l_gripper' are aliases for the link frames
    % 'r_hand_dh_frame' and 'l_hand_dh_frame', where their origins are placed on the axis
    % of their parent joints. The link names are describing the "virtual joints" that are
    % used to construct the dynamic model of the iCub-Robot in the visualizer.
    % Further details about the definitions of the joint, link and frame names are
    % available in the iCub Model Naming Conventions under,
    %    <http://wiki.icub.org/wiki/ICub_Model_naming_conventions>.
    %
    % Link names which are related to a specific parent joint of the robot model:
    %                                                                     idx:        | parent joints:
    joint_lnk_names = { 'root_link'; ...                                %  1,         | --,
                        'r_hip_1'; 'r_lower_leg'; 'r_sole'; ...         %  2,  3,  4, | r_hip_pitch, r_knee, r_foot* --> r_foot_ft_sensor,
                        'l_hip_1'; 'l_lower_leg'; 'l_sole'; ...         %  5,  6,  7, | l_hip_pitch, l_knee, l_foot* --> l_foot_ft_sensor,
                        'neck_1'; ...                                   %  8,         | neck_pitch,
                        'r_shoulder_1'; 'r_elbow_1'; 'r_gripper'; ...   %  9, 10, 11, | r_shoulder_pitch, r_elbow, r_hand_dh_frame** --> r_hand* --> r_wrist_yaw,
                        'l_shoulder_1'; 'l_elbow_1'; 'l_gripper'; ...   % 12, 13, 14, | l_shoulder_pitch, l_elbow, l_hand_dh_frame** --> l_hand* --> l_wrist_yaw,
                        'com' };                                        % 15          | --
                                                                        %             |
                                                                        %             | (* ... link; ** ... link frame)

    % Set of joint-pair indexes to describe the configuration of the iCub-robot's
    % skeleton. Each pair of joints is connected with a rigid link (edge) to form
    % a kinematic chain of the robot. The index-pairs denotes the index-position
    % of the given joint names in the above joint name list. The index-set is used
    % to create a set of position parameters that describes the full configuration
    % of the robot system.
    %
    % Joint pair indexes for the x, y and z-positions (translations) in the 3D-space:
    %                       x1 x2   y1 y2   z1 z2
    joint_pair_idx = uint8([ 1  8    1  8    1  8;
                             1  2    1  2    1  2;
                             2  3    2  3    2  3;
                             3  4    3  4    3  4;
                             1  5    1  5    1  5;
                             5  6    5  6    5  6;
                             6  7    6  7    6  7;
                             8  9    8  9    8  9;
                             9 10    9 10    9 10;
                            10 11   10 11   10 11;
                             8 12    8 12    8 12;
                            12 13   12 13   12 13;
                            13 14   13 14   13 14]);

    % List with constant scale factors to define the sizes of the shapes (patches) for
    % the links (edges) of the robot's skeleton. They form the shape of the robot's body:
    shape_geom.size_sf = [0.07   0.03;
                          0.04   0.02;
                          0.03   0.02;
                          0.025  0.02;
                          0.04   0.02;
                          0.03   0.02;
                          0.025  0.02;
                          0.03   0.02;
                          0.025  0.02;
                          0.02   0.02;
                          0.03   0.02;
                          0.025  0.02;
                          0.02   0.02];

    % Connection matrix to define which vertices are to connect for creating the shapes
    % of the links or the shapes of the feets. Each row represents one polygon (rectangle):
    shape_geom.faces = uint8([1 2 3 4;
                              1 4 8 5;
                              5 8 7 6;
                              7 3 2 6;
                              2 6 5 1;
                              3 7 8 4]);

    % Joint indices of those joints, where the left and the right foot is connected to:
    foot_geom.joints = uint8([4 7]);

    % Base size values for the feets of the robot:
    foot_geom.base_sz.width  = 0.025;
    foot_geom.base_sz.height = 0.015;
    %foot_geom.base_sz = [0.025 0.015]; % optional

    % List with additional size values to define the foot dimensions:
    %                       x:     y:  z:
    %                      len    wid hgt
    foot_geom.shape_ds = [-0.002   0   0; % up back
                          -0.002   0   0;
                          -0.01    0   0; % bottom back (back length)
                          -0.01    0   0;
                           0.025   0   0; % up front
                           0.025   0   0;
                           0.11    0   0; % bottom front (front length)
                           0.11    0   0];

    % Create the body model of the animated robot in the simulation:
    sim_body = WBM.wbmSimBody(joint_lnk_names, joint_pair_idx);

    % Geometry properties for the shape of the robot's body:
    sim_body.shape_geom = shape_geom;
    sim_body.foot_geom  = foot_geom;

    % Draw properties for the body of the simulated robot:
    sim_body.draw_prop.joints.color     = WBM.wbmColor.orange;
    sim_body.draw_prop.links.color      = 'black';
    sim_body.draw_prop.shape.face_color = WBM.wbmColor.royalblue4;
    sim_body.draw_prop.shape.edge_color = WBM.wbmColor.royalblue4;

    % Create the configuration object for the WBM-Simulator:
    sim_config = WBM.genericSimConfig('iCub-Simulator:', sim_body);

    % Environment settings for the animated scene:
    sim_config.environment.origin_pt_size = 4.5;
    % Light scene:
    sim_config.environment.ground_color   = WBM.wbmColor.papayawhip;

    % Dark scene (optional):
    %sim_config.robot_body.draw_prop.links.color      = WBM.wbmColor.seashell4;
    %sim_config.robot_body.draw_prop.shape.edge_color = WBM.wbmColor.steelblue;
    %sim_config.robot_body.draw_prop.shape.face_color = WBM.wbmColor.steelblue;

    %sim_config.environment.background_color_opt = 'black';
    %sim_config.environment.ground_color         = WBM.wbmColor.snow;
    %sim_config.environment.ground_edge_color    = 'none';
    %sim_config.environment.origin_pt_color      = WBM.wbmColor.violetred;
end
