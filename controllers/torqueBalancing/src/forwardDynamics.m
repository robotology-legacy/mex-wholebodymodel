function dchi = forwardDynamics(t,chi,MODEL,INIT_CONDITIONS)
%FORWARDDYNAMICS computes the forward dynamics of constrained floating
%                base robots. 
%
% Format: [dchi,GRAPHICS] = FORWARDDYNAMICS(t,chi,MODEL,INIT_CONDITIONS)
%
% Inputs:  - current time t [s];
%          - state vector chi [13+4*ndof x 1];
%          - MODEL: it is a structure defining the robot model;        
%          - INIT_CONDITIONS: it is a structure containing initial conditions
%                             for forward dynamics integration.
%
% Output:  - state vector derivative dchi [13+4*ndof x1];
%
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% global variables
global stanceFootForce state prevTimeIndex;

% update waitbar
if ~MODEL.disableVisForGraphics
    % report current time in the waitbar's message field
    waitbar(t/MODEL.CONFIG.tEnd,MODEL.wait,['Current time: ',sprintf('%0.3f',t),' [s]'])
end

% if the robot is performing yoga movements, update initial informations
% according to the current state
if strcmp(MODEL.CONFIG.demo_type,'yoga')  
    [MODEL,INIT_CONDITIONS] = finiteStateMachineYoga(t,MODEL,INIT_CONDITIONS);
end

%% Demux state vector inside the structure STATE
% robot degrees of freedom
ndof                  = MODEL.ndof;
% robot and floating base configuration
chi_robot             = chi(1:(13+2*ndof));
% demux robot and floating base configuration
STATE                 = robotState(chi_robot,MODEL);
w_R_b                 = STATE.w_R_b;
w_omega_b             = STATE.w_omega_b;
qt_b                  = STATE.qt_b;
dx_b                  = STATE.dx_b;
dqj                   = STATE.dqj;
qj                    = STATE.qj;
x_b                   = STATE.x_b;

%% Update the robot state (in case wbm_wrappers are used in optimized mode)
wbm_setWorldFrame(w_R_b,x_b,[0 0 -9.81]')
wbm_updateState(qj,dqj,[dx_b;w_omega_b]);

%% Robot dynamics is stored inside the structure DYNAMICS
DYNAMICS                = robotDynamics(STATE,MODEL);
Jc                      = DYNAMICS.Jc;
M                       = DYNAMICS.M;
h                       = DYNAMICS.h;
H                       = DYNAMICS.H;
m                       = M(1,1);

%% Control gains
GAINS                   = MODEL.GAINS;

%% Motors configuration (in case motor dynamics is considered in the model)
if MODEL.CONFIG.use_SEA
    xi                      = chi(14+2*ndof:13+3*ndof);
    dxi                     = chi(14+3*ndof:end);
    STATE.xi                = xi;
    STATE.dxi               = dxi; 
    [B_xi,KS,KD,damping_xi] = addElasticJoints(MODEL);
    DYNAMICS.B_xi           = B_xi;
    DYNAMICS.KS             = KS;
    DYNAMICS.KD             = KD;
    % add control gains for motor velocities
    GAINS.damping_xi        = damping_xi;
end

%% Robot Forward kinematics (stored into structure FORKINEMATICS)
FORKINEMATICS           = robotForKinematics(STATE,DYNAMICS);
poseRFoot_ang           = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang           = FORKINEMATICS.poseLFoot_ang;
xCoM                    = FORKINEMATICS.xCoM;

%% Joint limits check
if MODEL.CONFIG.use_jointLimits
    jointLimitsCheck(qj,t);
end

%% CoM and joints trajectory generator
if MODEL.CONFIG.use_ikinSolver
    % find in the vector "MODEL.REFERENCES.t" the index of the time step 
    % which is before the current time t
    index      = sum(MODEL.REFERENCES.t<t);
    nextIndex  = index+1;
    % if the next time index is exactly t, take the values without any
    % interpolation
    if MODEL.REFERENCES.t(nextIndex) == t
        TRAJECTORY.jointReferences.qjRef   = MODEL.REFERENCES.qj(:,nextIndex);
        TRAJECTORY.jointReferences.dqjRef  = MODEL.REFERENCES.dqj(:,nextIndex);
        TRAJECTORY.jointReferences.ddqjRef = MODEL.REFERENCES.ddqj(:,nextIndex);
    else
        % otherwise, interpolate the values to find the current step
        TRAJECTORY.jointReferences.qjRef   = linearInterpolation(MODEL.REFERENCES.t(nextIndex),t,MODEL.REFERENCES.t(index), ...
                                                                 MODEL.REFERENCES.qj(:,nextIndex),MODEL.REFERENCES.qj(:,index));
        TRAJECTORY.jointReferences.dqjRef  = linearInterpolation(MODEL.REFERENCES.t(nextIndex),t,MODEL.REFERENCES.t(index), ...
                                                                 MODEL.REFERENCES.dqj(:,nextIndex),MODEL.REFERENCES.dqj(:,index));
        TRAJECTORY.jointReferences.ddqjRef = linearInterpolation(MODEL.REFERENCES.t(nextIndex),t,MODEL.REFERENCES.t(index), ...
                                                                 MODEL.REFERENCES.ddqj(:,nextIndex),MODEL.REFERENCES.ddqj(:,index));
    end
else
    % in case ikinsolver is not used
    TRAJECTORY.jointReferences.ddqjRef     = zeros(ndof,1);
    TRAJECTORY.jointReferences.dqjRef      = zeros(ndof,1);
    TRAJECTORY.jointReferences.qjRef       = MODEL.REFERENCES.qjRef;
end
% CoM desired trajectory
TRAJECTORY.desired_x_dx_ddx_CoM            = trajectoryGenerator(MODEL.REFERENCES.xCoMRef,t,MODEL.CONFIG);

%% Initialize robot controller 
% initialize controller
CONTROLLER      = runController(GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,INIT_CONDITIONS,STATE,MODEL);
% control torques. Recall that if motor dynamics is considered, tau_xi == tau_motor/eta 
tau_xi          = CONTROLLER.tau_xi;
% contact forces
fc              = CONTROLLER.fc;

%% FORWARD DYNAMICS (dchi) COMPUTATION
b_omega_w       = transpose(w_R_b)*w_omega_b;
dq_b            = dquat(qt_b,b_omega_w);
nu              = [dx_b;dq_b;dqj];
if MODEL.CONFIG.use_SEA
    dnu         = M\(transpose(Jc)*fc + [zeros(6,1);(KS*(xi-qj)+KD*(dxi-dqj))]-h);
    % robot and floating base state derivative   
    dchi_robot  = [nu;dnu]; 
    % motor derivative
    ddxi        = B_xi\(tau_xi+KS*(qj-xi)+KD*(dqj-dxi));
    % total state derivative
    dchi        = [dchi_robot;dxi;ddxi];
else
    dnu         = M\(transpose(Jc)*fc + [zeros(6,1);tau_xi]-h);
    % total state derivative
    dchi        = [nu;dnu];
end

%% Update global variables
if strcmp(MODEL.CONFIG.demo_type,'yoga')
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
if MODEL.CONFIG.visualize_robot_simulator_ONLINE && ~MODEL.disableVisForGraphics
    iDyntreeSimulator(t,chi,MODEL)
end
% store parameters for visualizing integration results
if MODEL.CONFIG.enable_visualTool
    % resize force vector for 1 foot balancing 
    if sum(MODEL.CONFIG.feet_on_ground) == 1
        fc = [fc;zeros(6,1)];
    end
    % find index of the closest t greater than the current time in 'reduced time' vector
    timeTot       = MODEL.timeTot;
    currTimeIndex = sum(ones(1,length(timeTot))) - sum(timeTot > t);
    % first of all, check if the index has been updated correctly 
    if currTimeIndex > prevTimeIndex
        % if this is the first loop (t == tStart), initialize the structure GRAPHICS
        if currTimeIndex == 1
            % load values from .mat file
            GRAPHICS = matfile('./media/storedValuesFwdDyn.mat','Writable',true);
            if MODEL.CONFIG.use_SEA
                GRAPHICS.dxi(:,currTimeIndex)     = dxi;
                GRAPHICS.dxi_ref(:,currTimeIndex) = CONTROLLER.dxi_ref;
            end
            GRAPHICS.qj(:,currTimeIndex)        = qj;
            GRAPHICS.qjRef(:,currTimeIndex)     = TRAJECTORY.jointReferences.qjRef;
            GRAPHICS.xCoM(:,currTimeIndex)      = xCoM;
            GRAPHICS.xCoMDes(:,currTimeIndex)   = TRAJECTORY.desired_x_dx_ddx_CoM(:,1);            
            GRAPHICS.poseFeet(:,currTimeIndex)  = [poseLFoot_ang;poseRFoot_ang];
            GRAPHICS.H(:,currTimeIndex)         = H;
            GRAPHICS.HRef(:,currTimeIndex)      = [m*TRAJECTORY.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
            GRAPHICS.fc(:,currTimeIndex)        = fc;
            GRAPHICS.tau_xi(:,currTimeIndex)    = tau_xi;
            GRAPHICS.timeIndex(currTimeIndex,1) = currTimeIndex;
            % update time index
            prevTimeIndex                       = currTimeIndex;
        else    
            % in this case, just interpolate all variables to find the correct
            % value at reduced time step. Load previous value from .mat file:
            GRAPHICS = matfile('./media/storedValuesFwdDyn.mat','Writable',true);
            % then, interpolate to find the desired value (using classical
            % formula y = m*x + q; 
            if MODEL.CONFIG.use_SEA
                GRAPHICS.dxi(:,currTimeIndex)     = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),dxi,GRAPHICS.dxi(:,prevTimeIndex));
                GRAPHICS.dxi_ref(:,currTimeIndex) = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),CONTROLLER.dxi_ref,GRAPHICS.dxi_ref(:,prevTimeIndex));
            end
            GRAPHICS.qj(:,currTimeIndex)        = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),qj,GRAPHICS.qj(:,prevTimeIndex));
            GRAPHICS.qjRef(:,currTimeIndex)     = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),TRAJECTORY.jointReferences.qjRef,GRAPHICS.qjRef(:,prevTimeIndex));
            GRAPHICS.xCoM(:,currTimeIndex)      = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),xCoM,GRAPHICS.xCoM(:,prevTimeIndex));
            GRAPHICS.xCoMDes(:,currTimeIndex)   = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),TRAJECTORY.desired_x_dx_ddx_CoM(:,1),GRAPHICS.xCoMDes(:,prevTimeIndex));
            GRAPHICS.poseFeet(:,currTimeIndex)  = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),[poseLFoot_ang;poseRFoot_ang],GRAPHICS.poseFeet(:,prevTimeIndex));
            GRAPHICS.H(:,currTimeIndex)         = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),H,GRAPHICS.H(:,prevTimeIndex));
            GRAPHICS.HRef(:,currTimeIndex)      = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),[m*TRAJECTORY.desired_x_dx_ddx_CoM(:,2);zeros(3,1)],GRAPHICS.HRef(:,prevTimeIndex));
            GRAPHICS.fc(:,currTimeIndex)        = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),fc,GRAPHICS.fc(:,prevTimeIndex));
            GRAPHICS.tau_xi(:,currTimeIndex)    = linearInterpolation(t,timeTot(currTimeIndex),timeTot(prevTimeIndex),tau_xi,GRAPHICS.tau_xi(:,prevTimeIndex));
            GRAPHICS.timeIndex(currTimeIndex,1) = currTimeIndex;
            % update time index
            prevTimeIndex                       = currTimeIndex;
        end
    end
end
end
