function ContFig = visMain(t,chi,params)
%VISMAIN is the main function for visualize the results of iCub forward
%        dynamics integration in MATLAB.
%   VISMAIN visualizes some outputs from the forward dynamics 
%   integration (e.g. the robot state, contact forces, control torques...). 
%   Also, for the "stack of task" controller, it is possible to visualize 
%   the linearization results (both stability and gains tuning).
%
%   [] = VISMAIN(t,chi,params) takes as input the integration time t, the
%   robot state Chi and the structure params containing all the utility 
%   parameters. The output is a counter for the automatic correction of
%   figures numbers in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup parameters
ndof  = params.ndof;
set(0,'DefaultFigureWindowStyle','Docked');

%% FORWARD DYNAMICS (basic parameters)
% joints
qj            = zeros(ndof,length(t));
dqj           = zeros(ndof,length(t));
qjRef         = zeros(ndof,length(t));
dqjRef        = zeros(ndof,length(t));
ddqjRef       = zeros(ndof,length(t));
ddqjNonLin    = zeros(ndof,length(t));

% contact forces and torques
fc            = zeros(6*params.numConstraints,length(t));
f0            = zeros(6*params.numConstraints,length(t));
tau           = zeros(ndof,length(t));
normTau       = zeros(length(t),1);

% forward kinematics
xCoM          = zeros(3,length(t));
poseFeet      = zeros(14,length(t));
CoP           = zeros(4,length(t));
LfootOri      = zeros(3,length(t));
RfootOri      = zeros(3,length(t));
H             = zeros(6,length(t));
HRef          = zeros(6,length(t));

% generate the vectors from forward dynamics
for time = 1:length(t)
    
[~,visual]          = forwardDynamics(t(time), chi(time,:)', params);

% joints
qj(:,time)          = visual.qj;
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
 
if  params.numConstraints == 2 
    
CoP(3,time)         = -visual.fc(11)/visual.fc(9);
CoP(4,time)         =  visual.fc(10)/visual.fc(9);
end

% left foot orientation
PoseLFoot              = visual.poseFeet(1:7);
[~,RotLFoot]           = frame2posrot(PoseLFoot);
[~,LfootOri(:,time)]   = parametrization(RotLFoot);

% right foot orientation
PoseRFoot              = visual.poseFeet(8:end);
[~,RotRFoot]           = frame2posrot(PoseRFoot);
[~,RfootOri(:,time)]   = parametrization(RotRFoot);
end

%% COMPOSED PARAMETERS
HErr                = H-HRef;
qjErr               = qj - qjRef;
dqjErr              = dqj - dqjRef;

%% Basic visualization (forward dynamics integration results)
if params.visualize_integration_results == 1
    
params.ContFig = visIntegrationRes(t,params,xCoM,poseFeet,LfootOri,RfootOri,fc,f0,normTau,CoP,HErr);
end

%% Joints positions and position error
if params.visualize_joints_dynamics == 1 
      
params.ContFig = visJointDynamics(t,params,qj,qjRef);
end

%% Linearization results (stability and gains tuning)
if params.linearize_for_gains_tuning == 1 || params.linearize_for_stability_analysis == 1
    
params.ContFig = visLinResults(t,params,qjErr,dqjErr,ddqjNonLin,ddqjRef);     
end

ContFig = params.ContFig;
set(0,'DefaultFigureWindowStyle','Normal');

end
