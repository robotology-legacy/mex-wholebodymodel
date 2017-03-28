function dchi = forwardDynamics(t,chi,MODEL,INIT_CONDITIONS)
%FORWARDDYNAMICS computes the forward dynamics of constrained floating
%                base robots. 
%
% Format: [dchi,GRAPHICS] = FORWARDDYNAMICS(t,chi,MODEL,INIT_CONDITIONS)
%
% Inputs:  - current time t [s];
%          - state vector chi [13+4*ndof x 1];
%          - MODEL is a structure defining the robot model;
%          - INIT_CONDITIONS is a structure containing initial conditions
%            for integration.
%
% Output:  - state vector derivative dchi [13+4*ndof x1];
%
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% global variables
global stanceFootForce state prevTimeIndex;

% import utilities from WBM-Class
import WBM.utilities.dquat;

% update waitbar
waitbar(t/MODEL.CONFIG.tEnd,MODEL.wait)

% robot degrees of freedom
ndof = MODEL.ndof;

% if the robot is performing yoga movements, update initial informations
% according to the current state
if strcmp(MODEL.CONFIG.demo_type,'yoga')  
    [MODEL_updated,INIT_CONDITIONS_updated] = finiteStateMachine(t,MODEL,INIT_CONDITIONS);
else
    INIT_CONDITIONS_updated = INIT_CONDITIONS;
    MODEL_updated           = MODEL;
end

%% Demux state vector inside the structure STATE
% robot and floating base configuration
chi_robot             = chi(1:(13+2*ndof));
% demux robot and floating base configuration
STATE                 = robotState(chi_robot,MODEL_updated);
w_R_b                 = STATE.w_R_b;
w_omega_b             = STATE.w_omega_b;
qt_b                  = STATE.qt_b;
dx_b                  = STATE.dx_b;
dqj                   = STATE.dqj;
qj                    = STATE.qj;
x_b                   = STATE.x_b;
% motors configuration
xi                    = chi(14+2*ndof:13+3*ndof);
dxi                   = chi(14+3*ndof:end);
% add motors configuration to the STATE structure
STATE.xi              = xi;
STATE.dxi             = dxi;

%% Set update robot state (in case wbm_wrappers are used in optimized mode)
wbm_setWorldFrame(w_R_b,x_b,[0 0 -9.81]')
wbm_updateState(qj,dqj,[dx_b;w_omega_b]);

%% Robot dynamics is stored inside the structure DYNAMICS
DYNAMICS                = robotDynamics(STATE,MODEL_updated);
Jc                      = DYNAMICS.Jc;
M                       = DYNAMICS.M;
h                       = DYNAMICS.h;
H                       = DYNAMICS.H;
m                       = M(1,1);
% add motors dynamics and joints elasticity 
[B_xi,KS,KD,damping_xi] = addElasticJoints(MODEL_updated);
DYNAMICS.B_xi           = B_xi;
DYNAMICS.KS             = KS;
DYNAMICS.KD             = KD;

%% Robot Forward kinematics (stored into structure FORKINEMATICS)
FORKINEMATICS           = robotForKinematics(STATE,DYNAMICS);
poseRFoot_ang           = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang           = FORKINEMATICS.poseLFoot_ang;
xCoM                    = FORKINEMATICS.xCoM;

%% Joint limits check
if MODEL_updated.CONFIG.use_jointLimits
    jointLimitsCheck(qj,t);
end

%% CoM and joints trajectory generator
TRAJECTORY.jointReferences.ddqjRef = zeros(ndof,1);
TRAJECTORY.jointReferences.dqjRef  = zeros(ndof,1);
TRAJECTORY.jointReferences.qjRef   = INIT_CONDITIONS_updated.qjRef;
TRAJECTORY.desired_x_dx_ddx_CoM    = trajectoryGenerator(INIT_CONDITIONS_updated.xCoMRef,t,MODEL_updated.CONFIG);

%% Control gains
GAINS                   = INIT_CONDITIONS_updated.GAINS;
% add control gains for motor velocities
GAINS.damping_xi        = damping_xi;

%% Initialize robot controller 
% initialize controller
CONTROLLER      = runController(GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,INIT_CONDITIONS_updated,STATE,MODEL_updated);
% recall that tau_xi == tau_motor/eta 
tau_xi          = CONTROLLER.tau_xi;
% contact forces
fc              = CONTROLLER.fc;

%% FORWARD DYNAMICS (dchi) COMPUTATION
b_omega_w       = transpose(w_R_b)*w_omega_b;
dq_b            = dquat(qt_b,b_omega_w);
nu              = [dx_b;dq_b;dqj];
dnu             = M\(transpose(Jc)*fc + [zeros(6,1);(KS*(xi-qj)+KD*(dxi-dqj))]-h);
% robot and floating base state derivative   
dchi_robot      = [nu;dnu];
% motor derivative
ddxi            = B_xi\(tau_xi+KS*(qj-xi)+KD*(dqj-dxi));
% total state derivative
dchi            = [dchi_robot;dxi;ddxi];

%% Update global variables
if strcmp(MODEL_updated.CONFIG.demo_type,'yoga')
    if state == 8
        % two feet balacing, moving towards right foot 
        stanceFootForce = fc(9);
    else
        % all other states
        stanceFootForce = fc(3);
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%  VISUALIZATION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% for debugging it is possible to visualize robot movements during
% numerical integration. WARNING: it will considerably slow down the
% simulation!
if MODEL_updated.CONFIG.visualize_robot_simulator_ONLINE && ~MODEL_updated.disableVisForGraphics
    iDyntreeSimulator(t,chi,MODEL_updated)
end
% Store parameters for visualizing integration results
if MODEL_updated.CONFIG.visualize_integration_results
    % resize force vector for 1 foot balancing 
    if sum(MODEL_updated.CONFIG.feet_on_ground) == 1
        fc = [zeros(6,1);fc];
    end
    % find index of the closest time greater than the current time in reduced time vector
    timeTot       = MODEL_updated.timeTot;
    currTimeIndex = sum(ones(1,length(timeTot))) - sum(timeTot > t);
    % first of all, check if the index has been updated correctly 
    if currTimeIndex > prevTimeIndex
        % if this is the first loop, initialize the structure GRAPHICS
        if currTimeIndex == 1
            % Load values from .mat file
            GRAPHICS = matfile('./media/storedValues.mat','Writable',true);
            GRAPHICS.dxi(:,currTimeIndex)       = dxi;
            GRAPHICS.dxi_ref(:,currTimeIndex)   = CONTROLLER.dxi_ref;
            GRAPHICS.qj(:,currTimeIndex)        = qj;
            GRAPHICS.qjRef(:,currTimeIndex)     = TRAJECTORY.jointReferences.qjRef;
            GRAPHICS.xCoM(:,currTimeIndex)      = xCoM;
            GRAPHICS.poseFeet(:,currTimeIndex)  = [poseLFoot_ang;poseRFoot_ang];
            GRAPHICS.H(:,currTimeIndex)         = H;
            GRAPHICS.HRef(:,currTimeIndex)      = [m*TRAJECTORY.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
            GRAPHICS.fc(:,currTimeIndex)        = fc;
            GRAPHICS.tau_xi(:,currTimeIndex)    = tau_xi;
            GRAPHICS.xCoMDes(:,currTimeIndex)   = TRAJECTORY.desired_x_dx_ddx_CoM(:,1);
            GRAPHICS.timeIndex(currTimeIndex,1) = currTimeIndex;
            % update time index
            prevTimeIndex      = currTimeIndex;
        else    
            % in this case, just interpolate all variables to find the correct
            % value at reduced time step. Load previous value from .mat file
            GRAPHICS = matfile('./media/storedValues.mat','Writable',true);
            % then, interpolate to find the desired value (using classical
            % formula y = m*x + q;        
            GRAPHICS.dxi(:,currTimeIndex)       = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),dxi,GRAPHICS.dxi(:,prevTimeIndex));
            GRAPHICS.dxi_ref(:,currTimeIndex)   = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),CONTROLLER.dxi_ref,GRAPHICS.dxi_ref(:,prevTimeIndex));
            GRAPHICS.qj(:,currTimeIndex)        = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),qj,GRAPHICS.qj(:,prevTimeIndex));
            GRAPHICS.qjRef(:,currTimeIndex)     = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),TRAJECTORY.jointReferences.qjRef,GRAPHICS.qjRef(:,prevTimeIndex));
            GRAPHICS.xCoM(:,currTimeIndex)      = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),xCoM,GRAPHICS.xCoM(:,prevTimeIndex));
            GRAPHICS.poseFeet(:,currTimeIndex)  = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),[poseLFoot_ang;poseRFoot_ang],GRAPHICS.poseFeet(:,prevTimeIndex));
            GRAPHICS.H(:,currTimeIndex)         = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),H,GRAPHICS.H(:,prevTimeIndex));
            GRAPHICS.HRef(:,currTimeIndex)      = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),[m*TRAJECTORY.desired_x_dx_ddx_CoM(:,2);zeros(3,1)],GRAPHICS.HRef(:,prevTimeIndex));
            GRAPHICS.fc(:,currTimeIndex)        = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),fc,GRAPHICS.fc(:,prevTimeIndex));
            GRAPHICS.tau_xi(:,currTimeIndex)    = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),tau_xi,GRAPHICS.tau_xi(:,prevTimeIndex));
            GRAPHICS.xCoMDes(:,currTimeIndex)   = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),TRAJECTORY.desired_x_dx_ddx_CoM(:,1),GRAPHICS.xCoMDes(:,prevTimeIndex));
            GRAPHICS.timeIndex(currTimeIndex,1) = currTimeIndex;
            % update time index
            prevTimeIndex                      = currTimeIndex;
        end
    end
end
end
