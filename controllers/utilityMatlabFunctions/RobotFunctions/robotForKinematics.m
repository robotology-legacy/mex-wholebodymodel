function FORKINEMATICS = robotForKinematics(STATE,DYNAMICS)
%ROBOTFORKINEMATICS uses the forward kinematics to define the pose and
%                   velocity of some cartesian points, such as the CoM or
%                   the contacts.
%       forKinematics = ROBOTFORKINEMATICS(state,dynamics) takes as
%       an input the current state of the robot, which is defined in the
%       structure STATE and the structure DYNAMICS which contains the robot
%       dynamics.
%       The output is the structure FORKINEMATICS which contains pose and
%       velocity at CoM, feet, and so on.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% State parameters
RotBase        = STATE.RotBase;
PosBase        = STATE.PosBase;
qj             = STATE.qj;
Nu             = STATE.Nu;

% Dynamics parameters
JCoM           = DYNAMICS.JCoM;
Jc             = DYNAMICS.Jc;

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
[TLfootOri,oriLfoot]             = parametrization(RotBaseLfoot);
[TRfootOri,oriRfoot]             = parametrization(RotBaseRfoot);
PoseLFootEul                     = [posLfoot; oriLfoot'];
PoseRFootEul                     = [posRfoot; oriRfoot'];

%% Define the output structure
FORKINEMATICS.xCoM                     = xCoM;
FORKINEMATICS.dxCoM                    = dxCoM;
FORKINEMATICS.LFootPoseQuat            = PoseLFootQuat;
FORKINEMATICS.RFootPoseQuat            = PoseRFootQuat;
FORKINEMATICS.LFootPoseEul             = PoseLFootEul;
FORKINEMATICS.RFootPoseEul             = PoseRFootEul;
FORKINEMATICS.VelFeet                  = VelFeet;
FORKINEMATICS.TLfoot                   = [eye(3) zeros(3) ; zeros(3) TLfootOri];
FORKINEMATICS.TRfoot                   = [eye(3) zeros(3) ; zeros(3) TRfootOri];

end
