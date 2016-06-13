function ContFig = initVisualizer(t,chi,config)
%INITVISUALIZER is the main function for visualize the results of iCub forward
%               dynamics integration in MATLAB.
%   INITVISUALIZER visualizes some outputs from the forward dynamics 
%   integration (e.g. the robot state, contact forces, control torques...). 
%   Also, for the "stack of task" controller, it is possible to visualize 
%   the linearization results (both stability and gains tuning).
%
%   ContFig = INITVISUALIZER(t,chi,config) takes as input the integration 
%   time T, the robot state CHI and the structure CONFIG containing all the
%   utility parameters. The output is a counter for the automatic correction
%   of figures numbers in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup parameters
ndof                          = config.ndof;
config.visualizeLinearization = 0;
config.allowVisualization     = 0;

if config.visualize_stability_analysis_results == 1 || config.visualize_gains_tuning_results == 1

config.visualizeLinearization = 1;
end

if config.linearizeJointSp  == 1 && config.visualizeLinearization == 1
    
config.allowVisualization = 1;
end

%% ROBOT SIMULATOR
if config.visualize_robot_simulator == 1
    
config.ContFig = visualizeSimulation(t,chi,config);
end

%% FORWARD DYNAMICS (basic parameters)
if config.visualize_integration_results == 1  || config.visualize_joints_dynamics == 1 || config.allowVisualization == 1

config.wait = waitbar(0,'Generating the plots...');
set(0,'DefaultFigureWindowStyle','Docked');

% joints
qj            = zeros(ndof,length(t));
qjInit        = zeros(ndof,length(t));
dqj           = zeros(ndof,length(t));
qjRef         = zeros(ndof,length(t));
dqjRef        = zeros(ndof,length(t));
ddqjRef       = zeros(ndof,length(t));
ddqjNonLin    = zeros(ndof,length(t));

% contact forces and torques
fc            = zeros(6*config.numConstraints,length(t));
f0            = zeros(6*config.numConstraints,length(t));
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
    
[~,visual]          = forwardDynamics(t(time), chi(time,:)', config);

% joints
qj(:,time)          = visual.qj;
qjInit(:,time)      = config.qjInit;
dqj(:,time)         = visual.dqj;
qjRef(:,time)       = visual.JointRef.qjRef;
dqjRef(:,time)      = visual.JointRef.dqjRef;
ddqjRef(:,time)     = visual.JointRef.ddqjRef;
ddqjNonLin(:,time)  = visual.ddqjNonLin;

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
 
if  config.numConstraints == 2 
    
CoP(3,time)         = -visual.fc(11)/visual.fc(9);
CoP(4,time)         =  visual.fc(10)/visual.fc(9);
end

end

delete(config.wait)

% composed parameters
HErr                   = H-HRef;
qjErr                  = qj-qjRef;
dqjErr                 = dqj-dqjRef;

%% Basic visualization (forward dynamics integration results)
if config.visualize_integration_results == 1
    
config.ContFig = visualizeForwardDyn(t,config,xCoM,poseFeet,fc,f0,normTau,CoP,HErr);
end

%% Joints positions and position error
if config.visualize_joints_dynamics == 1 
      
config.ContFig = visualizeJointDynamics(t,config,qj,qjRef);
end

%% Linearization results (stability and gains tuning)
if config.linearizeJointSp == 1
    
config.ContFig = visualizeLinearization(t,config,qjErr,dqjErr,ddqjNonLin,ddqjRef);     
end

ContFig = config.ContFig;
set(0,'DefaultFigureWindowStyle','Normal');

end
