function [dchi,visualization] = forwardDynamics(t,chi,INIT_CONDITIONS)
%FORWARDDYNAMICS computes the forward dynamics of floating base robots
%                given initial conditions, contact forces and torques. 
%
% [dchi,visualization] = FORWARDDYNAMICS(t,chi,CONFIG) takes as input the
% current time step, t; the state, chi [13+2*ndof x 1]; the structure
% CONFIG which contains initial conditions and user-defined parameters.
% The output are the vector to be integrated, dchi [13+2*ndof x1] and the
% structure visualization which is used for plotting results in the visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
global stanceFootForce CoMError state;
import WBM.utilities.dquat;
waitbar(t/INIT_CONDITIONS.CONFIG.tEnd,INIT_CONDITIONS.wait)

%% RIGENERATE INITIAL CONDITIONS ACCORDING TO CURRENT STATE
if strcmp(INIT_CONDITIONS.CONFIG.demo_type,'yoga')  
    % call the finite state machine
    INIT_CONDITIONS = stateMachine(t,INIT_CONDITIONS);
end

%% Update robot and motor state
% control gains
gain                  = INIT_CONDITIONS.gainsInit;
% robot configuration
ndof                  = INIT_CONDITIONS.CONFIG.ndof;
chi_robot             = chi(1:(13+2*ndof));
% motor configuration
xi                    = chi(14+2*ndof:13+3*ndof);
dxi                   = chi(14+3*ndof:end);
ELASTICITY            = addElasticJoints(INIT_CONDITIONS.CONFIG);

%% CoM and joints trajectory generator
trajectory.jointReferences.ddqjRef = zeros(ndof,1);
trajectory.jointReferences.dqjRef  = zeros(ndof,1);
trajectory.jointReferences.qjRef   = INIT_CONDITIONS.qjRef;
trajectory.desired_x_dx_ddx_CoM    = trajectoryGenerator(INIT_CONDITIONS.xCoMRef,t,INIT_CONDITIONS.CONFIG);

%% Robot State
STATE                 = robotState(chi_robot,INIT_CONDITIONS.CONFIG);
w_R_b                 = STATE.w_R_b;
w_omega_b             = STATE.w_omega_b;
qt_b                  = STATE.qt_b;
dx_b                  = STATE.dx_b;
dqj                   = STATE.dqj;
qj                    = STATE.qj;
x_b                   = STATE.x_b;

%% Set the robot state (for wbm wrappers in 'optimized mode')
wbm_setWorldFrame(w_R_b,x_b,[0 0 -9.81]')
wbm_updateState(qj,dqj,[dx_b;w_omega_b]);

%% Robot Dynamics
DYNAMICS              = robotDynamics(STATE,INIT_CONDITIONS.CONFIG);
Jc                    = DYNAMICS.Jc;
M                     = DYNAMICS.M;
h                     = DYNAMICS.h;
H                     = DYNAMICS.H;
m                     = M(1,1);

%% Robot Forward kinematics
FORKINEMATICS         = robotForKinematics(STATE,DYNAMICS);
poseRFoot_ang         = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang         = FORKINEMATICS.poseLFoot_ang;
xCoM                  = FORKINEMATICS.xCoM;

%% Joint limits check
% jointLimitsCheck(qj,t);

%% Torque balancing controller
controlParam  = runController(gain,trajectory,DYNAMICS,FORKINEMATICS,INIT_CONDITIONS,STATE,dxi,xi,ELASTICITY);

% recall that tau_xi = tau_motor/eta 
tau_xi        = controlParam.tau_xi;
fc            = controlParam.fc;

% update global variables
if strcmp(INIT_CONDITIONS.CONFIG.demo_type,'yoga')
    if state == 8
        stanceFootForce = fc(9);
    else
        stanceFootForce = fc(3);
    end
    CoMError            = xCoM-INIT_CONDITIONS.xCoMRef;
end

%% State derivative (dchi) computation
b_omega_w       = transpose(w_R_b)*w_omega_b;
dq_b            = dquat(qt_b,b_omega_w);
nu              = [dx_b;dq_b;dqj];
dnu             = M\(Jc'*fc + [zeros(6,1);(ELASTICITY.KS*(xi-qj)+ELASTICITY.KD*(dxi-dqj))]-h);
% state derivative   
dchi_robot      = [nu;dnu];
% motor derivative
ddxi            = ELASTICITY.B_xi\(tau_xi+ELASTICITY.KS*(qj-xi)+ELASTICITY.KD*(dqj-dxi));
% total state derivative
dchi            = [dchi_robot;dxi;ddxi];

%% ROBOT ONLINE SIMULATION
if INIT_CONDITIONS.CONFIG.visualize_robot_simulator_ONLINE 
    INIT_CONDITIONS.VISUALIZER.jointPos = iDynTree.JointPosDoubleArray(INIT_CONDITIONS.VISUALIZER.model);
    INIT_CONDITIONS.VISUALIZER.jointPos.fromMatlab(qj);
    % compute the world_H_base that correspond to the specified joints 
    INIT_CONDITIONS.VISUALIZER.odom = iDynTree.SimpleLeggedOdometry();
    INIT_CONDITIONS.VISUALIZER.odom.setModel(INIT_CONDITIONS.VISUALIZER.model);
    INIT_CONDITIONS.VISUALIZER.odom.updateKinematics(INIT_CONDITIONS.VISUALIZER.jointPos);    
    if sum(INIT_CONDITIONS.CONFIG.feet_on_ground) == 2
    
        INIT_CONDITIONS.VISUALIZER.odom.init('l_sole','l_sole');
    
    elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 1 && INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 0
    
        INIT_CONDITIONS.VISUALIZER.odom.init('l_sole','l_sole');
    
    elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 0 && INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 1
    
        INIT_CONDITIONS.VISUALIZER.odom.init('r_sole','r_sole');
    end    
    INIT_CONDITIONS.VISUALIZER.viz.modelViz(0).setPositions(INIT_CONDITIONS.VISUALIZER.odom.getWorldLinkTransform(INIT_CONDITIONS.VISUALIZER.model.getDefaultBaseLink()),INIT_CONDITIONS.VISUALIZER.jointPos);
    INIT_CONDITIONS.VISUALIZER.viz.draw();
end

%% Parameters for visualization
visualization.xi          = xi;
visualization.dxi         = dxi;
visualization.dxi_ref     = controlParam.dxi_ref;
visualization.qj          = qj;
visualization.jointRef    = trajectory.jointReferences;
visualization.xCoM        = xCoM;
visualization.poseFeet    = [poseLFoot_ang;poseRFoot_ang];
visualization.H           = H;
visualization.HRef        = [m*trajectory.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
visualization.fc          = fc;
visualization.f0          = controlParam.f0;
visualization.tau_xi      = tau_xi;
visualization.xCoMDes     = trajectory.desired_x_dx_ddx_CoM(:,1);

end
