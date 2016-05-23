function forKinematics = robotForKinematics(state,dynamics)
%ROBOTFORKINEMATICS uses the forward kinematics to define the pose and
%                   velocity at some points of interest, such as the CoM or
%                   the feet.
%         forKinematics = ROBOTFORKINEMATICS(state,dynamics) takes as 
%         an input the current state of the robot, which is defined in the
%         structure STATE and the structure DYNAMICS which contains the robot 
%         dynamics.
%         The output is the structure FORKINEMATICS which
%         contains pose and velocity at CoM, feet, and so on.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% State parameters
RotBase        = state.RotBase;
PosBase        = state.PosBase;
qj             = state.qj;
Nu             = state.Nu;

% Dynamics parameters
JCoM           = dynamics.JCoM;
Jc             = dynamics.Jc;

%% FORWARD KINEMATICS
% feet pose (quaternions), CoM position
PoseLFootQuat                    = wbm_forwardKinematics(RotBase,PosBase,qj,'l_sole');
PoseRFootQuat                    = wbm_forwardKinematics(RotBase,PosBase,qj,'r_sole');
CoM                              = wbm_forwardKinematics(RotBase,PosBase,qj,'com');
xCoM                             = CoM(1:3);

% feet velocity, CoM velocity
VelFeet                          = Jc*Nu;
dCoM                             = JCoM*Nu;
dxCoM                            = dCoM(1:3);

%% Feet orientation using Euler angles
% feet current position and orientation (rotation matrix)
[posLfoot,RotBaseLfoot]          = frame2posrot(PoseLFootQuat);
[posRfoot,RotBaseRfoot]          = frame2posrot(PoseRFootQuat);

% orientation is parametrized with euler angles
[TLfoot,oriLfoot]                = parametrization(RotBaseLfoot);
[TRfoot,oriRfoot]                = parametrization(RotBaseRfoot);
PoseLFootEul                     = [posLfoot; oriLfoot'];
PoseRFootEul                     = [posRfoot; oriRfoot'];

%% Define the output structure
forKinematics.xCoM                     = xCoM;
forKinematics.dxCoM                    = dxCoM;
forKinematics.LFootPoseQuat            = PoseLFootQuat;
forKinematics.RFootPoseQuat            = PoseRFootQuat;
forKinematics.LFootPoseEul             = PoseLFootEul;
forKinematics.RFootPoseEul             = PoseRFootEul;
forKinematics.VelFeet                  = VelFeet;
forKinematics.TLfoot                   = [eye(3) zeros(3) ; zeros(3) TLfoot];
forKinematics.TRfoot                   = [eye(3) zeros(3) ; zeros(3) TRfoot];

end
