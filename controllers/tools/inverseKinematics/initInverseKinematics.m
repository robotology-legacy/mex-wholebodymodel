function REFERENCES = initInverseKinematics(MODEL,INIT_CONDITIONS)
%INITINVERSEKINEMATICS contains the initial conditions for inverse kinematics
%                      integration of floating base robots using MATLAB.
%
% Format:  REFERENCES = INITINVERSEKINEMATICS(MODEL,INIT_CONDITIONS)
%
% Inputs:  - MODEL: it is a structure defining the robot model;        
%          - INIT_CONDITIONS: it is a structure containing initial conditions
%                             for forward dynamics integration.
%
% Output:  - REFERENCES: it is a structure containing the joints reference 
%                        trajectory, velocity and acceleration. 
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% config parameters
pinv_tol               = MODEL.CONFIG.pinv_tol;
ndof                   = MODEL.ndof;
m                      = INIT_CONDITIONS.INITDYNAMICS.M(1,1);
xCoM                   = INIT_CONDITIONS.INITFORKINEMATICS.xCoM;

%% Define initial conditions for ikin integration
% this actually encompasses the fact that for all demos, the starting 
% reference velocity and acceleration are always zero, but the reference 
% joint/CoM position may be slightly different from the initial one, and
% this may cause the initial reference velocity to be different from zero.
% To avoid an undesired step reference at initial time, the following three
% task stack of task procedure is setted up.

% task #1: enforce contact constraints. This is done by computing Jc*nu = 0
% where Jc is the contact jacobian and nu is the robot + floating base
% velocity. 
constraintsDynamics    = zeros(6*length(MODEL.constraintLinkNames),1);
Jc                     = INIT_CONDITIONS.INITDYNAMICS.Jc;
pinvJc                 = pinv(Jc,pinv_tol);
NullJc                 = eye(ndof+6) - pinvJc*Jc;
% task #2: obtain a desired momentum dynamics. This is achieved comuting
% JH*nu = H_desired. the velocity obtained from this equation is then projected
% in the null space of the first task, NullJc.
JH                     = INIT_CONDITIONS.INITDYNAMICS.JH;
pinvJH                 = pinv(JH*NullJc,pinv_tol);
momentumDynamics       = [-m*(xCoM-MODEL.REFERENCES.xCoMRef);zeros(3,1)] -JH*pinvJc*constraintsDynamics;
NullJH                 = eye(ndof+6) - pinvJH*(JH*NullJc);
% task #3: postural task. The joint position is computed from the formula
% JP*nu = qj-qjRef and as the previous task, it is projected in the null
% space NullJh.
St                     = [zeros(ndof,6) eye(ndof)];
jointDynamics          = -(INIT_CONDITIONS.INITSTATE.qj-MODEL.REFERENCES.qjRef)-St*(pinvJc*constraintsDynamics+NullJc*pinvJH*momentumDynamics);
pinvJP                 = pinv(St*NullJc*NullJH,pinv_tol);

% task-base state velocity
nuThirdTask            = pinvJP*jointDynamics;
nuSecondTask           = pinvJH*momentumDynamics + NullJH*nuThirdTask;
nuFirstTask            = pinvJc*constraintsDynamics + NullJc*nuSecondTask;

% initial condition for ikin integration
chi_robotInit          = [INIT_CONDITIONS.chi_robotInit(1:(7+ndof)); nuFirstTask];

% set a waitbar and modify the default options
MODEL.wait = waitbar(0,'1','Name','Ikin integration in progress...',...
                     'CreateCancelBtn','setappdata(gcbf,''canceling'',1); delete(gcbf);');
set(MODEL.wait, 'Units', 'Pixels', 'Position', [800 500 365 100])

%% %%%%%%%%%%%%%%%%% INVERSE KINEMATICS INTEGRATION %%%%%%%%%%%%%%%%%%%% %%
% state machine is not available for ikin integration, because it is not
% possible to trigger the contact thresholds (ikin does not consider
% contact forces!). Ikin is available only for demo that does not require a
% state machine.
REFERENCES             = integrateInverseKinematics(MODEL,chi_robotInit);
delete(MODEL.wait)
disp('[ikinSolver]: inverse kinematics integration completed')
  
end
