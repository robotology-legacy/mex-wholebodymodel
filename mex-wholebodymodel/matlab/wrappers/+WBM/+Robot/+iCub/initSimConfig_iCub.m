function sim_config = initSimConfig_iCub()
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

	% List with constant scale factors that are related to the sizes of the patches
	% around the links (edges) of the robot's skeleton to build up the hull of the body:
	hull_geom.size_sf = [0.07   0.03;
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
	hull_geom.faces = uint8([1 2 3 4;
							 1 4 8 5;
              			   	 5 8 7 6;
              			   	 7 3 2 6;
              			   	 2 6 5 1;
              			   	 3 7 8 4]);

	% Joint indices of the joints for the left and the right foot:
    foot_geom.joints = uint8([4 7]);

    % Base size values to create the feets for the robot:
    foot_geom.base_sz.width  = 0.025;
    foot_geom.base_sz.height = 0.015;

    % list with additional size values for the foot dimensions:
    %          			  x:    y:  z:
    %					 len   wid hgt
    foot_geom.shape_ds = [0     0   0;
		            	  0     0   0;
		                  0     0   0;
		                  0     0   0;
		                  0.025 0   0; % up
		                  0.025 0   0;
		                  0.1   0   0; % bottom
		                  0.1   0   0];

	% draw properties for the animated robot:
	% define some new colors (source: <http://research.stowers-institute.org/efg/R/Color/Chart/ColorChart.pdf>):
	%maroon4      = [139  28  98] ./ 255;
	orange       = [255 165 0] ./ 255;
	
	%midnightblue = [ 25  25 112] ./ 255;
	%navyblue     = [  0   0 128] ./ 255;
	%darkblue = [0 0 139] ./ 255;
	royalblue4 =  [39 64 139] ./ 255;
	
	%lightgoldenrodyellow = [250 250 210] ./ 255;
	%lightsteelblue1  = [202 225 255] ./ 255;
	%lightyellow = [255 255 224] ./ 255;
	%moccasin = [255 228 181] ./ 255;
	%seashell = [255 245 238] ./ 255;
	%papayawhip = [255 239 213] ./ 255;
	antiquewhite = [250 235 215] ./ 255;


	sim_body = WBM.wbmSimBody(joint_names, joint_pair_idx);

	sim_body.hull_geometry = hull_geom;
	sim_body.foot_geometry = foot_geom;

	sim_body.draw_prop.joints.color     = orange;
	sim_body.draw_prop.links.color      = 'black';
	sim_body.draw_prop.hull.face_color = royalblue4;
	sim_body.draw_prop.hull.edge_color = royalblue4;
	sim_body.draw_prop.ground_color    = antiquewhite;

	% create the configuration object for the WBM-Simulator:
    sim_config = WBM.genericSimConfig(sim_body, 'iCub-Simulator:');
end
