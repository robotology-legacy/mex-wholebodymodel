function sim_config = setupSimConfig_iCub()
	% List of joint names for constructing the iCub-Robot in the visualizer:
	joint_names = { 'root_link'; ...
				 	'r_hip_1'; 'r_lower_leg'; 'r_sole'; ...
					'l_hip_1'; 'l_lower_leg'; 'l_sole'; ...
					'neck_1'; ...
					'r_shoulder_1'; 'r_elbow_1'; 'r_gripper'; ...
					'l_shoulder_1'; 'l_elbow_1'; 'l_gripper'; ...
					'com' };

	% Set of joint-pair indexes to describe the configuration of the iCub-robot's
	% skeleton. Each pair of joints is connected with a rigid link (edge) to form
	% a kinematic chain of the robot. The index-pairs denotes the index-position
	% of the given joint names in the above joint name list. The index-set is used
	% to create a set of position parameters that describes the full configuration
	% of the robot system.
	%
	% Joint pair indexes for the x, y and z-positions (translations) in the 3D-space:
	%				   x1 x2   y1 y2   z1 z2
	joint_pair_idx = {[ 1  8    1  8    1  8;
					    1  2    1  2    1  2;
						3  2    3  2    3  2;
						3  4    3  4    3  4;
						1  5    1  5    1  5;
						6  5    6  5    6  5;
						6  7    6  7    6  7;
						8  9    8  9    8  9;
					   	10  9   10  9   10  9;
					   	10 11   10 11   10 11;
						8 12    8 12    8 12;
					   	13 12   13 12   13 12;
					   	13 14   13 14   13 14 ]; % (*)

					  [ 1  8    1  8    1  8;
						1  2    1  2    1  2;
						3  2    3  2    3  2;
						3  4    3  4    3  4;
						1  5    1  5    1  5;
						5  6    5  6    5  6; % (**)
						6  7    6  7    6  7;
						9  8    9  8    9  8; % (**)
						9 10    9 10    9 10; % (**)
					   10 11   10 11   10 11;
						8 12    8 12    8 12;
					   13 12   13 12   13 12;
					   13 14   13 14   13 14 ]}; % (***)
	%   (*) ... first (base) configuration set.
	%  (**) ... order swaped for each pair.
	% (***) ... second configuration set.

	sim_body = WBM.wbmSimBody(joint_names, joint_pair_idx);

	% List with constant scale factors that are related to the sizes of the patches
	% around the links (edges) of the robot's skeleton to build up the hull of the body:
	sim_body.hull_size_sf = [0.07   0.03;
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
		
	% Connection matrix to define which vertices are to connect for creating the
	% hull of the link or the shape of the feets. Each row represents one polygon (rectangle):
	sim_body.hull_faces = uint8([1 2 3 4;
              			   		 1 4 8 5;
              			   		 5 8 7 6;
              			   		 7 3 2 6;
              			   		 2 6 5 1;
              			   		 3 7 8 4]);

	% joint indices of the joints for the left and the right foot:
    sim_body.foot_joints = uint8([4,7]);

    % list with additional size values for the foot dimensions:
    %          			x    y z
    sim_body.foot_ds = [0    0 0;
		                0    0 0;
		                0    0 0;
		                0    0 0;
		                0.03 0 0;
		                0.03 0 0;
		                0.03 0 0;
		                0.03 0 0];

    sim_config = WBM.genericSimConfig(sim_body, 'iCub-Simulator:');
end
