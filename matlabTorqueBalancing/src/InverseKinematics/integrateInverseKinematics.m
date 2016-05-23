function ikinParam = integrateInverseKinematics(config,ChiInit)
%INTEGRATEINVERSEKINEMATICS integrates the inverse kinematics of the robot iCub.
%                           It uses a fixed step integrator.
%                    
%   ikinParam = INTEGRATEINVERSEKINEMATICS(config,ChiInit) takes as input the
%   structure CONFIG which contains all the utility parameters, and the
%   initial state of the robot CHIINIT. The output is the structure IKINPARAM,
%   which contains the joint reference trajectory.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup initial parameters
tStep = config.ikin_integration_step;
t     = transpose(config.tStart:tStep:config.tEnd);
ndof  = config.ndof;

%% Initial conditions for inverse dynamics integration
dimTime             = length(t);
dimState            = length(ChiInit);
qj                  = zeros(ndof,dimTime);
dqj                 = qj;
ddqj                = dqj;
ChiIkin             = zeros(dimState,dimTime);
CoMTrajectoryError  = zeros(18,dimTime);
momentumError       = zeros(6,dimTime);

if config.numConstraints == 2

feetError           = zeros(12,dimTime);

elseif config.numConstraints == 1

feetError           = zeros(6,dimTime);
end
   
[dChiInit,visualizeIkinParam]   = inverseKinematics(t(1),ChiInit,config);

ChiIkin(:,1)                    = ChiInit;
qj(:,1)                         = ChiInit(8:7+ndof);
dqj(:,1)                        = ChiInit(14+ndof:end);
ddqj(:,1)                       = dChiInit(14+ndof:end);
CoMTrajectoryError(:,1)         = visualizeIkinParam.CoMTrajectoryError;
feetError(:,1)                  = visualizeIkinParam.feetError;         
momentumError(:,1)              = visualizeIkinParam.momentumError;      

%% Function to be integrated
integratedFunction              = @(t,Chi) inverseKinematics(t,Chi,config);

%% Euler forward integrator (fixed step)
for kk = 2:dimTime

% state at step k
ChiIkin(:,kk)   = ChiIkin(:,kk-1) + tStep.*integratedFunction(t(kk-1),ChiIkin(:,kk-1));
 
% joint references and visualization parameters
[dChiIkin,visualizeIkinParam] = inverseKinematics(t(kk),ChiIkin(:,kk),config); 
 
qj(:,kk)        = ChiIkin(8:7+ndof,kk);
dqj(:,kk)       = ChiIkin(14+ndof:end,kk);
ddqj(:,kk)      = dChiIkin(14+ndof:end);

CoMTrajectoryError(:,kk)         = visualizeIkinParam.CoMTrajectoryError;
feetError(:,kk)                  = visualizeIkinParam.feetError;         
momentumError(:,kk)              = visualizeIkinParam.momentumError;   
end

%% Joint trajectory and visualization parameters
ikinParam.qj                     = qj;
ikinParam.dqj                    = dqj;
ikinParam.ddqj                   = ddqj;
ikinParam.t                      = t;
ikinParam.feetError              = feetError;
ikinParam.CoMTrajectoryError     = CoMTrajectoryError;
ikinParam.momentumError          = momentumError;

end
    
