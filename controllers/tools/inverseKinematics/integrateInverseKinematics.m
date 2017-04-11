function IKIN = integrateInverseKinematics(CONFIG,chiInit)
%INTEGRATEINVERSEKINEMATICS integrates the inverse kinematics of the robot iCub.
%                           It uses a fixed step integrator.
%
% IKIN = INTEGRATEINVERSEKINEMATICS(CONFIG,chiInit) takes as input the
% structure CONFIG which contains all the utility parameters, and the
% initial state of the robot chiInit. The output is the structure IKIN,
% which contains the joint reference trajectory and visualization parameters.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup initial parameters
tStep = CONFIG.ikin_integration_step;
t     = transpose(CONFIG.tStart:tStep:CONFIG.tEnd);
ndof  = CONFIG.ndof;

%% Initial conditions for inverse dynamics integration
dimTime             = length(t);
dimState            = length(chiInit);
qj                  = zeros(ndof,dimTime);
dqj                 = qj;
ddqj                = dqj;
chiIkin             = zeros(dimState,dimTime);
CoMTrajectoryError  = zeros(18,dimTime);
momentumError       = zeros(6,dimTime);

if CONFIG.numConstraints == 2
    
    feetError           = zeros(12,dimTime);
    
elseif CONFIG.numConstraints == 1
    
    feetError           = zeros(6,dimTime);
end

[dChiInit,visualizeIkinParam]   = inverseKinematics(t(1),chiInit,CONFIG);

chiIkin(:,1)                    = chiInit;
qj(:,1)                         = chiInit(8:7+ndof);
dqj(:,1)                        = chiInit(14+ndof:end);
ddqj(:,1)                       = dChiInit(14+ndof:end);
CoMTrajectoryError(:,1)         = visualizeIkinParam.CoMTrajectoryError;
feetError(:,1)                  = visualizeIkinParam.feetError;
momentumError(:,1)              = visualizeIkinParam.momentumError;

%% Function to be integrated
integratedFunction              = @(t,Chi) inverseKinematics(t,Chi,CONFIG);

%% Euler forward integrator (fixed step)
for kk = 2:dimTime
    
    % state at step k
    chiIkin(:,kk)   = chiIkin(:,kk-1) + tStep.*integratedFunction(t(kk-1),chiIkin(:,kk-1));
    
    % joint references and visualization parameters
    [dChiIkin,visualizeIkinParam] = inverseKinematics(t(kk),chiIkin(:,kk),CONFIG);
    
    qj(:,kk)                      = chiIkin(8:7+ndof,kk);
    dqj(:,kk)                     = chiIkin(14+ndof:end,kk);
    ddqj(:,kk)                    = dChiIkin(14+ndof:end);
    
    CoMTrajectoryError(:,kk)      = visualizeIkinParam.CoMTrajectoryError;
    feetError(:,kk)               = visualizeIkinParam.feetError;
    momentumError(:,kk)           = visualizeIkinParam.momentumError;
end

%% Joint trajectory and visualization parameters
IKIN.chiIkin                = chiIkin;
IKIN.qj                     = qj;
IKIN.dqj                    = dqj;
IKIN.ddqj                   = ddqj;
IKIN.t                      = t;
IKIN.feetError              = feetError;
IKIN.CoMTrajectoryError     = CoMTrajectoryError;
IKIN.momentumError          = momentumError;

end

