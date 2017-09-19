function [icub_body, jnt_names] = setupBody_iCub_atf()
    %% Define the effectors and the chains for the body of the iCub-Robot:
    % Sources:
    %   [1] Yarp-WholeBodyInterface: <https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface.ini>
    %   [2] iCub Model Naming Convention: <http://wiki.icub.org/wiki/ICub_Model_naming_conventions>

    % Effectors (list of all chain names):
    chn_names = {'ROBOT_TORSO'; 'ROBOT_LEFT_ARM'; 'ROBOT_RIGHT_ARM'; 'ROBOT_LEFT_LEG'; 'ROBOT_RIGHT_LEG'};

    % Chains (lists with all joints of each chain):
    % Note: The joint names must be named as defined in the naming convention [2] and
    %       must be set in the same order as defined in the configuration file in [1].
    %                                                                                                                                               joint-idx:
    %                                                                                                                                                bgn  end
    robot_jnts_torso     = {'torso_yaw'; 'torso_roll'; 'torso_pitch'};                                                                             %   1..3,
    robot_jnts_left_arm  = {'l_shoulder_pitch'; 'l_shoulder_roll'; 'l_shoulder_yaw'; 'l_elbow'; 'l_wrist_prosup'; 'l_wrist_pitch'; 'l_wrist_yaw'}; %   4..10,
    robot_jnts_right_arm = {'r_shoulder_pitch'; 'r_shoulder_roll'; 'r_shoulder_yaw'; 'r_elbow'; 'r_wrist_prosup'; 'r_wrist_pitch'; 'r_wrist_yaw'}; %  11..17,
    robot_jnts_left_leg  = {'l_hip_pitch'; 'l_hip_roll'; 'l_hip_yaw'; 'l_knee'; 'l_ankle_pitch'; 'l_ankle_roll'};                                  %  18..23,
    robot_jnts_right_leg = {'r_hip_pitch'; 'r_hip_roll'; 'r_hip_yaw'; 'r_knee'; 'r_ankle_pitch'; 'r_ankle_roll'};                                  %  24..29

    % Joint name list:
    % Note: The order of the joint names is set as defined in the listing of ROBOT_MEX_WBI_TOOLBOX in [1].
    jnt_names = vertcat(robot_jnts_torso, robot_jnts_left_arm, robot_jnts_right_arm, robot_jnts_left_leg, robot_jnts_right_leg);

    chn_idx = [1 3; 4 10; 11 17; 18 23; 24 29]; % joint index pairs to define the start and the end of each chain ...
    jnt_idx = (1:29).';

    icub_body = WBM.wbmBody(chn_names, chn_idx, jnt_names, jnt_idx);
end
