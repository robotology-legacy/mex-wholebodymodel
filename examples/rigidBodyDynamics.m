%% In this example we will show how use the mex-wholeBodyModel to compute
%% the dynamics quantitites of a rigid body

%% cleanup the session
clear
close all;

%% initialise mexWholeBodyModel using the rigidBody.urdf file.
%% Check the rigidBody.urdf for comments on how the inertial
%% parameters are encoded in the URDF file
%% The function is searching the file in the current working
%% directory, so make sure that you are launching the script
%% in the directory where rigidBody.urdf is found
wbm_modelInitialiseFromURDF('rigidBody.urdf');

%% the number of (internal) dofs is 0 for a rigid body
%% we set the state to some random values, just to show how to
%% get the dynamics quantities
w_R_b   = eye(3,3); % rotation matrix that transforms a vector in the base frame to the world frame
x_b     = [1;2;3]; % position of the link frame origin wrt to the world frame
qj      = zeros(0,1);  % joint positions
dqj     = zeros(0,1); % joint velocities
grav    = [0;0;-9.8]; % gravity in world frame
dx_b    = [0.4;0.5;0.6]; % derivative in the position of the link frame origin wrt to the world frame
omega_W = [0.4;0.5;0.2]; % angular velocity of base frame

%% Set the state
wbm_setWorldFrame(w_R_b,x_b,grav);
wbm_updateState(qj,dqj,[dx_b;omega_W]);

% We compute the mass matrix:
M = wbm_massMatrix();

% and the generalized bias forces (coriolis + gravity forces)
h = wbm_generalisedBiasForces();

% We can check them by printing them:
disp(M)
disp(h)

% we can also get the forward kinematics of any frame with respect to the world
% in this case it will be just return the x_b and w_R_b value that we set in the
% wbm_setWorldFrame method, as root_link is the base frame, but in more complex
% models it can compute the forward kinematics for any frame
world_T_frame = wbm_forwardKinematics('root_link');

% As it is possible to notice by printing it, world_T_frame is a vector of 7 elements,
% where the first three elements are the position of the frame origin wrt to the world frame,
% and the last 4 elements are the quaternion corresponding to the rotation between the frame and the world frame
% for more info, check wbm_forwardKinematics options
disp(world_T_frame)

