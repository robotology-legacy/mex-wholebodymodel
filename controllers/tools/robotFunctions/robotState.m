function STATE = robotState(chi,CONFIG)
%ROBOTSTATE demux the current state of the robot to get the base pose, base
%           velocity, joint position and joint velocity.
%
% STATE = ROBOTSTATE(chi,CONFIG) takes as an input the vector integration 
% vector chi [13+2ndof x 1], and the structure CONFIG which contains all the
% user-defined parameters. The output is the structure STATE which contains 
% the following variables:
%
% x_b            the cartesian position of the base (R^3)
% qt_b           the quaternion describing the orientation of the base (global parametrization of SO(3))
% qj             the joint positions (R^ndof)
% dx_b           the cartesian velocity of the base (R^3)
% w_omega_b      the velocity describing the orientation of the base (SO(3))
% dqj            the joint velocities (R^ndof)
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
% CONFIG.demux.baseOrientationType sets the base orientation in stateDemux.m
% as positions + quaternions (1) or transformation matrix (0)
import WBM.utilities.frame2posRotm;
CONFIG.demux.baseOrientationType = 1;

%% STATE DEMUX
[basePose,qj,baseVelocity,dqj]   = stateDemux(chi,CONFIG);

% Base position and orientation (in quaternions)
x_b                              = basePose(1:3,:);
qt_b                             = basePose(4:7,:);

% Normalize quaternions to avoid numerical errors
% qt_b                             = qt_b/norm(qt_b);

% Base velocity; conversion of the base orientation into a rotation matrix;
% state velocity
dx_b                             = baseVelocity(1:3,:);
w_omega_b                        = baseVelocity(4:6,:);
[~,w_R_b]                        = frame2posRotm(basePose);
nu                               = [dx_b;w_omega_b;dqj];

%% Define the output structure
STATE.qj                         = qj;
STATE.dqj                        = dqj;
STATE.nu                         = nu;
STATE.basePose                   = basePose;
STATE.w_R_b                      = w_R_b;
STATE.x_b                        = x_b;
STATE.dx_b                       = dx_b;
STATE.qt_b                       = qt_b;
STATE.w_omega_b                  = w_omega_b;

end

