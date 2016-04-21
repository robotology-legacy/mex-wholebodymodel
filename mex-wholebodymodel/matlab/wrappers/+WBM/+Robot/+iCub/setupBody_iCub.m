function icub_body = setupBody_iCub()
    %% Define the effectors and the chains for the body of the iCub-Robot:
    % Sources:
    %   [1] Yarp-WholeBodyInterface: <https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface.ini>
    %   [2] iCub Model Naming Convention: <http://wiki.icub.org/wiki/ICub_Model_naming_conventions>

    % Effectors (list of all chain names):
    chain_names = {'ROBOT_TORSO'; 'ROBOT_LEFT_ARM'; 'ROBOT_RIGHT_ARM'; 'ROBOT_LEFT_LEG'; 'ROBOT_RIGHT_LEG'};

    % Chains (lists with all joints of each chain):
    % Note: The joint names must be the same as defined in configuration file in [1,2].
    %                                                                                                                 joint-idx:
    %                                                                                                                  bgn  end
    robot_torso_joints     = {'torso_yaw'; 'torso_roll'; 'torso_pitch'};                                             %   1..3,
    robot_left_arm_joints  = {'l_shoulder_pitch'; 'l_shoulder_roll'; 'l_shoulder_yaw'; 'l_elbow'; 'l_wrist_prosup'}; %   4..8,
    robot_right_arm_joints = {'r_shoulder_pitch'; 'r_shoulder_roll'; 'r_shoulder_yaw'; 'r_elbow'; 'r_wrist_prosup'}; %   9..13,
    robot_left_leg_joints  = {'l_hip_pitch'; 'l_hip_roll'; 'l_hip_yaw'; 'l_knee'; 'l_ankle_pitch'; 'l_ankle_roll'};  %  14..19,
    robot_right_leg_joints = {'r_hip_pitch'; 'r_hip_roll'; 'r_hip_yaw'; 'r_knee'; 'r_ankle_pitch'; 'r_ankle_roll'};  %  20..25

    % Joint name list:
    % Note: The order of the joint names is set as defined in the listing of ROBOT_MEX_WBI_TOOLBOX in [1].
    joint_names = vertcat(robot_torso_joints, robot_left_arm_joints, robot_right_arm_joints, robot_left_leg_joints, robot_right_leg_joints);

    chain_idx = [1 3; 4 8; 9 13; 14 19; 20 25]; % joint index pairs to define the start and the end of each chain ...
    joint_idx = (1:25)';

    icub_body = WBM.wbmBody(chain_names, chain_idx, joint_names, joint_idx);
end
