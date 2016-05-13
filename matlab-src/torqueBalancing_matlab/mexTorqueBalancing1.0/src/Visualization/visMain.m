%% visMain
% visualizes some user-defined parameters such as contact forces, torques,
% CoM error. It can also analize the linearization results and generates a
% demo of the robot's movements
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [] = visMain(t,chi,params)
%% Demo generation
ndof = params.ndof;
set(0,'DefaultFigureWindowStyle','Docked');

%% Graphics generation 
%  generates all the parameters defined in forwardDynamicsSoT.m
qj            = zeros(ndof,length(t));
qjInit        = zeros(ndof,length(t));
qjRef         = zeros(ndof,length(t));
dqjRef        = zeros(ndof,length(t));
ddqjRef       = zeros(ndof,length(t));
qjErr         = zeros(params.ndof,length(t));
dqj           = zeros(params.ndof,length(t));
ddqjNonLin    = zeros(params.ndof,length(t));
norm_qjErr    = zeros(1,length(t));
fc            = zeros(6*params.numConstraints,length(t));
f0            = zeros(6*params.numConstraints,length(t));
tau           = zeros(params.ndof,length(t));
norm_tau      = zeros(1,length(t));
xCoM          = zeros(3,length(t));
error_CoM     = zeros(3,length(t));
pos_feet      = zeros(14,length(t));
CoP           = zeros(4,length(t));
phi_lfoot     = zeros(3,length(t));
phi_rfoot     = zeros(3,length(t));
H             = zeros(6,length(t));
Href          = zeros(6,length(t));
norm_HErr     = zeros(1,length(t));

for time = 1:length(t)
    
[~,visual]          = forwardDynamics(t(time), chi(time,:)', params);

qj(:,time)          = visual.qj;
qjRef(:,time)       = visual.JointRef.qjRef;
dqjRef(:,time)      = visual.JointRef.dqjRef;
ddqjRef(:,time)     = visual.JointRef.ddqjRef;
dqj(:,time)         = visual.dqj;
ddqjNonLin(:,time)  = visual.ddqjNonLin;
qjInit(:,time)      = params.qjInit;
qjErr(:,time)       = visual.qj- visual.JointRef.qjRef;
xCoM(:,time)        = visual.xCoM;
pos_feet(:,time)    = visual.pos_feet;
fc(:,time)          = visual.fc;
f0(:,time)          = visual.f0;
tau(:,time)         = visual.tau;
error_CoM(:,time)   = visual.error_com;
H(:,time)           = visual.H;
Href(:,time)        = visual.Href;

% square norms
norm_tau(time)   = norm(visual.tau);
norm_qjErr(time) = norm(visual.qj-params.qjInit);
norm_HErr(time)  = norm(visual.H-visual.Href);
 
CoP(1,time)      = -visual.fc(5)/visual.fc(3);
CoP(2,time)      =  visual.fc(4)/visual.fc(3);

if  params.numConstraints == 2 
    
CoP(3,time)      = -visual.fc(11)/visual.fc(9);
CoP(4,time)      =  visual.fc(10)/visual.fc(9);
end

% left foot orientation
quat_lFoot             = visual.pos_feet(1:7);
[~,Rot_lFoot]          = frame2posrot(quat_lFoot);
[~,phi_lfoot(:,time)]  = parametrization(Rot_lFoot);

% right foot orientation
quat_rFoot             = visual.pos_feet(8:end);
[~,Rot_rFoot]          = frame2posrot(quat_rFoot);
[~,phi_rfoot(:,time)]  = parametrization(Rot_rFoot);
end

%% Basic visualization
if params.visualize_integration_plot == 1
    
visIntegrationPlot(t,params,xCoM,pos_feet,phi_lfoot,phi_rfoot,fc,f0,tau,error_CoM,norm_tau,CoP,norm_qjErr,norm_HErr);

%% Joints positions and position error
caseJointPos = 0;
caseJointErr = 0;

if params.visualize_joints_position == 1 
      
    caseJointPos = 1;  
end

if params.visualize_joints_error == 1 
    
    caseJointErr = 1;
end

visJointPosAndErr(t,caseJointPos,caseJointErr,qj,qjRef,qjErr);
end

%% Linearized system plot
if params.linearize_for_gains_tuning == 1 || params.linearize_for_stability_analysis == 1
    
dqjTilde  = dqj-dqjRef;
visLinResults(t,params,qj,qjRef,dqjTilde,ddqjNonLin,ddqjRef);     
end

set(0,'DefaultFigureWindowStyle','Normal');

end
