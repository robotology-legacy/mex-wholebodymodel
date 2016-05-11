%% integrateInverseKin
% calculates the desired joints position and velocity by integrating the desired
% joints acceleration obratined with a 'stack of task inverse kinematics'. 
% It uses forward euler integration algorithm.
% The output is:
%
% ikinParam             this is a structure containing the desired joints reference
%                       trajetory and the integration time
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function ikinParam = integrateInverseKin(params,ChiInit)
%% Define the integration time (fixed step)
t      = (params.tStart:params.ikin_integration_step:params.tEnd)';
tStep  = params.ikin_integration_step;
ndof   = params.ndof;

%% Initial conditions for joints positions, velocities and accelerations
[dChiInit, visualizeIkinParam] = inverseKinematicsFunction(t(1), ChiInit, params);

% initial condition for errors visualization
ErrorsInit       = visualizeIkinParam.H_Feet_Error;
CoMTrajInit      = visualizeIkinParam.CoMTrajectory;

%% Setup integration
integratedFunction  =  @(time,ChiIkin) inverseKinematicsFunction(time, ChiIkin, params);

% setup matrix dimensions
dimTime     = length(t);
dimState    = length(ChiInit);
qj          = zeros(ndof,dimTime);
dqj         = qj;
ddqj        = dqj;
Chi         = zeros(dimState,dimTime);
Chi(:,1)    = ChiInit;
qj(:,1)     = ChiInit(8:7+ndof);
dqj(:,1)    = ChiInit(14+ndof:end);
ddqj(:,1)   = dChiInit(14+ndof:end);

if params.numConstraints == 2

H_Feet_Error        = zeros(18,dimTime);

elseif params.numConstraints == 1

H_Feet_Error        = zeros(12,dimTime);
end
   
CoMTrajectory         = zeros(18,dimTime);
H_Feet_Error(:,1)     = ErrorsInit;
CoMTrajectory(:,1)    = CoMTrajInit;

%% Euler forward integrator
for kk = 2:dimTime
    
% State at the current step
 Chi(:,kk)   = Chi(:,kk-1) + tStep.*integratedFunction(t(kk-1),Chi(:,kk-1));
 
% desired joints position, velocity and acceleration
[dChi, visualizeIkinParam] = integratedFunction(t(kk), Chi(:,kk)); 
 
 qj(:,kk)       = Chi(8:7+ndof,kk);
 dqj(:,kk)      = Chi(14+ndof:end,kk);
 ddqj(:,kk)     = dChi(14+ndof:end);

% errors visualization
 H_Feet_Error(:,kk)    = visualizeIkinParam.H_Feet_Error;
 CoMTrajectory(:,kk)   = visualizeIkinParam.CoMTrajectory;
end

%% Joints trajectory definition
ikinParam.qj         = qj;
ikinParam.dqj        = dqj;
ikinParam.ddqj       = ddqj;
ikinParam.t          = t;

%% Parameters for visualization
ikinParam.H_Feet_Error    = H_Feet_Error;
ikinParam.CoMTrajectory   = CoMTrajectory;

end
