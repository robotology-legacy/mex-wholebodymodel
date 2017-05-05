function INIT_CONDITIONS  = addStepReference(MODEL,INIT_CONDITIONS_OLD) 
%ADDSTEPREFERENCE modifies the upper body initial posture of the robot, but not 
%                 update the references. This causes the robot upper body
%                 joints to have a step.
%
% Format:  INIT_CONDITIONS  = ADDSTEPREFERENCE(MODEL,INIT_CONDITIONS_OLD) 
%
% Inputs:  - MODEL: it is a structure defining the robot model;        
%          - INIT_CONDITIONS_OLD: it is a structure containing initial conditions
%                                 for forward dynamics integration.
%
% Output:  - INIT_CONDITIONS: it is a structure containing initial conditions
%                             for forward dynamics integration.
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% config parameters
delta                = MODEL.CONFIG.stepAmplitude;
qjInit               = INIT_CONDITIONS_OLD.INITSTATE.qj;

%% New robot configuration
% modify the upper body joint position by a small delta
qjInit(1:13)         = INIT_CONDITIONS_OLD.INITSTATE.qj(1:13) + delta*pi/180;

% update robot state using the new initial conditions  
wbm_updateState(qjInit,INIT_CONDITIONS_OLD.INITSTATE.dqj,[INIT_CONDITIONS_OLD.INITSTATE.dx_b;INIT_CONDITIONS_OLD.INITSTATE.w_omega_b]);

% fixing the world reference frame w.r.t. the foot on ground position
[x_bInit,w_R_bInit]  = wbm_getWorldFrameFromFixLnk(MODEL.constraintLinkNames{1},qjInit);
wbm_setWorldFrame(w_R_bInit,x_bInit,[0 0 -9.81]')

% get current base pose
[basePoseInit,~,~,~] = wbm_getState();

% new initial robot state (floating base + joints)
INIT_CONDITIONS.chi_robotInit = [basePoseInit; qjInit; [INIT_CONDITIONS_OLD.INITSTATE.dx_b;INIT_CONDITIONS_OLD.INITSTATE.w_omega_b]; INIT_CONDITIONS_OLD.INITSTATE.dqj];

%% Update state demux, dynamics and forward kinematics
% initial state
INIT_CONDITIONS.INITSTATE         = robotState(INIT_CONDITIONS.chi_robotInit,MODEL);
% initial dynamics
INIT_CONDITIONS.INITDYNAMICS      = robotDynamics(INIT_CONDITIONS.INITSTATE,MODEL);
% initial forward kinematics
INIT_CONDITIONS.INITFORKINEMATICS = robotForKinematics(INIT_CONDITIONS.INITSTATE,INIT_CONDITIONS.INITDYNAMICS); 

disp('[StepReference]: upper body initial posture has been updated')
    
end