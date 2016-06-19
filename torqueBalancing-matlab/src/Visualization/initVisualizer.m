function figureCont = initVisualizer(t,chi,CONFIG)
%INITVISUALIZER is the main function for visualize the results of iCub forward
%               dynamics integration in MATLAB.
%
%   INITVISUALIZER visualizes some outputs from the forward dynamics 
%   integration (e.g. the robot state, contact forces, control torques...). 
%   Also, for the "stack of task" controller, it is possible to visualize 
%   the linearization results (both soundness of linearization and gains tuning).
%
%   figureCont = INITVISUALIZER(t,chi,config) takes as input the integration 
%   time T, the robot state CHI and the structure CONFIG containing all the
%   utility parameters. The output is a counter for the automatic correction
%   of figures numbers in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Config parameters
ndof                             = CONFIG.ndof;
CONFIG.visualizeLinearization    = 0;
CONFIG.allowLinVisualization     = 0;
initState                        = CONFIG.initState;

if CONFIG.gains_tuning  == 1 || CONFIG.linearizationDebug == 1
    
CONFIG.allowLinVisualization  = 1;
end

%% ROBOT SIMULATOR
if CONFIG.visualize_robot_simulator == 1
    
CONFIG.figureCont = visualizeSimulation(t,chi,CONFIG);
end

%% FORWARD DYNAMICS (basic parameters)
if CONFIG.visualize_integration_results == 1  || CONFIG.visualize_joints_dynamics == 1 || CONFIG.allowVisualization == 1 
    
CONFIG.wait = waitbar(0,'Generating the plots...');
set(0,'DefaultFigureWindowStyle','Docked');

% joints
qj            = zeros(ndof,length(t));
qjInit        = zeros(ndof,length(t));
dqj           = zeros(ndof,length(t));
qjRef         = zeros(ndof,length(t));
dqjRef        = zeros(ndof,length(t));
ddqjRef       = zeros(ndof,length(t));
ddqjNonLin    = zeros(ndof,length(t));
ddqjLin       = zeros(ndof,length(t));

% contact forces and torques
fc            = zeros(6*CONFIG.numConstraints,length(t));
f0            = zeros(6*CONFIG.numConstraints,length(t));
tau           = zeros(ndof,length(t));
normTau       = zeros(length(t),1);

% forward kinematics
xCoM          = zeros(3,length(t));
poseFeet      = zeros(12,length(t));
CoP           = zeros(4,length(t));
H             = zeros(6,length(t));
HRef          = zeros(6,length(t));

% generate the vectors from forward dynamics

for time = 1:length(t)
    
[~,visual]          = forwardDynamics(t(time), chi(time,:)', CONFIG);

% joints
qj(:,time)          = visual.qj;
qjInit(:,time)      = initState.qj;
dqj(:,time)         = visual.dqj;
qjRef(:,time)       = visual.JointRef.qjRef;
dqjRef(:,time)      = visual.JointRef.dqjRef;
ddqjRef(:,time)     = visual.JointRef.ddqjRef;
ddqjNonLin(:,time)  = visual.ddqjNonLin;
if CONFIG.linearizationDebug == 1
ddqjLin(:,time)     = visual.ddqjLin;
end
% contact forces and torques
fc(:,time)          = visual.fc;
f0(:,time)          = visual.f0;
tau(:,time)         = visual.tau;
normTau(time)       = norm(visual.tau);

% forward kinematics
xCoM(:,time)        = visual.xCoM;
poseFeet(:,time)    = visual.poseFeet;
H(:,time)           = visual.H;
HRef(:,time)        = visual.HRef;

% centers of pressure at feet
CoP(1,time)         = -visual.fc(5)/visual.fc(3);
CoP(2,time)         =  visual.fc(4)/visual.fc(3);
 
if  CONFIG.numConstraints == 2 
    
CoP(3,time)         = -visual.fc(11)/visual.fc(9);
CoP(4,time)         =  visual.fc(10)/visual.fc(9);
end

end

delete(CONFIG.wait)

% composed parameters
HErr                = H-HRef;

%% Basic visualization (forward dynamics integration results)
if CONFIG.visualize_integration_results == 1
    
CONFIG.figureCont = visualizeForwardDyn(t,CONFIG,xCoM,poseFeet,fc,f0,normTau,CoP,HErr);
end

%% Joints positions and position error
if CONFIG.visualize_joints_dynamics == 1 
      
CONFIG.figureCont = visualizeJointDynamics(t,CONFIG,qj,qjRef);
end

%% Linearization results (soundness of linearization and gains tuning)
if CONFIG.allowLinVisualization == 1
    
CONFIG.figureCont = visualizeLinearization(t,CONFIG,ddqjNonLin,ddqjLin);     
end

figureCont = CONFIG.figureCont;
set(0,'DefaultFigureWindowStyle','Normal');

end
