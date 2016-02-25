function icub_body = setupBody_iCub()

	chain_names = {'ROBOT_TORSO'; 'ROBOT_LEFT_ARM'; 'ROBOT_RIGHT_ARM'; 'ROBOT_LEFT_LEG'; 'ROBOT_RIGHT_LEG'};

	robot_torso_joints = {'torso_yaw'; 'torso_roll'; 'torso_pitch'};
	robot_left_arm_joints = {'l_shoulder_pitch'; 'l_shoulder_roll'; 'l_shoulder_yaw'; 'l_elbow'; 'l_wrist_prosup'}; 					
	robot_right_arm_joints = {'r_shoulder_pitch'; 'r_shoulder_roll'; 'r_shoulder_yaw'; 'r_elbow'; 'r_wrist_prosup'};
	robot_left_leg_joints = {'l_hip_pitch'; 'l_hip_roll'; 'l_hip_yaw'; 'l_knee'; 'l_ankle_pitch'; 'l_ankle_roll'};
	robot_right_leg_joints = {'r_hip_pitch'; 'r_hip_roll'; 'r_hip_yaw'; 'r_knee'; 'r_ankle_pitch'; 'r_ankle_roll'};

	joint_names = vertcat(robot_torso_joints, robot_left_arm_joints, robot_right_arm_joints, robot_left_leg_joints, robot_right_leg_joints);

	chain_pos = [1 3; 4 8; 9 13; 14 19; 20 25];
	joint_idx = [1:25]';

	icub_body = WBM.wbmBody(chain_names, chain_pos, joint_names, joint_idx);
end
