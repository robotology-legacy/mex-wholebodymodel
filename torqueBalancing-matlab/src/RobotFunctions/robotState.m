function STATE = robotState(chi,CONFIG)
%ROBOTSTATE demux the current state of the robot to get the base pose, base
%           velocity, joint pose and joint velocity.
%
%         state = ROBOTSTATE(chi,config) takes as an input the vector that
%         comes from integration, CHI [13+2ndof x 1], and the structure CONFIG
%         which contains all the user defined parametes. The output is
%         the structure state which contains the following variables:
%
% PosBase        the cartesian position of the base (R^3)
% quatBase       the quaternion describing the orientation of the base (global parametrization of SO(3))
% qj             the joint positions (R^ndof)
% VelBase        the cartesian velocity of the base (R^3)
% omegaBaseWorld the velocity describing the orientation of the base (SO(3))
% dqj            the joint velocities (R^ndof)
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
% CONFIG.demux.baseOrientationType sets the base orientation in stateDemux.m 
% as positions + quaternions (1) or transformation matrix (0)
CONFIG.demux.baseOrientationType = 1;   

%% STATE DEMUX
[basePose,qj,baseVelocity,dqj]   = stateDemux(chi,CONFIG);
 
% Base position and orientation (in quaternions)
PosBase                          = basePose(1:3,:);
quatBase                         = basePose(4:7,:);

% Normalize quaternions to avoid numerical errors
%quatBase                        = quatBase/norm(quatBase);

% Base velocity; conversion of the base orientation into a rotation matrix; 
% state velocity
VelBase                          = baseVelocity(1:3,:);
omegaBaseWorld                   = baseVelocity(4:6,:);
[~,RotBase]                      = frame2posrot(basePose);
Nu                               = [VelBase;omegaBaseWorld;dqj];

%% Define the output structure
STATE.qj                         = qj;
STATE.dqj                        = dqj;
STATE.Nu                         = Nu;
STATE.basePose                   = basePose;
STATE.RotBase                    = RotBase;
STATE.PosBase                    = PosBase;
STATE.VelBase                    = VelBase;
STATE.quatBase                   = quatBase;
STATE.omegaBaseWorld             = omegaBaseWorld;

end









