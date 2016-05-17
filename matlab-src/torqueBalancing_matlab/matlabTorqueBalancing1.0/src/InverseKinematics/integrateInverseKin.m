function ikinParam = integrateInverseKin(params,ChiInit)
%INTEGRATEINVERSEKIN integrates the inverse kinematics of the robot iCub. It
%                    uses a fixed step integrator.
%                    
%   ikinParam = INTEGRATEINVERSEKIN(params,ChiInit) takes as input the
%   structure params which contains all the utility parameters, and the
%   initial state of the robot ChiInit. The output is the structure ikinParam,
%   which contains the joint reference trajectory.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup initial parameters
tStep = params.ikin_integration_step;
t     = transpose(params.tStart:tStep:params.tEnd);
ndof  = params.ndof;

%% Initial conditions for inverse dynamics integration
dimTime             = length(t);
dimState            = length(ChiInit);
qj                  = zeros(ndof,dimTime);
dqj                 = qj;
ddqj                = dqj;
ChiIkin             = zeros(dimState,dimTime);
CoMTrajectoryError  = zeros(18,dimTime);
momentumError       = zeros(6,dimTime);

if params.numConstraints == 2

feetError           = zeros(12,dimTime);

elseif params.numConstraints == 1

feetError           = zeros(6,dimTime);
end
   
[dChiInit,visualizeIkinParam] = inverseKinematicsFunction(t(1),ChiInit,params);

ChiIkin(:,1)                    = ChiInit;
qj(:,1)                         = ChiInit(8:7+ndof);
dqj(:,1)                        = ChiInit(14+ndof:end);
ddqj(:,1)                       = dChiInit(14+ndof:end);
CoMTrajectoryError(:,1)         = visualizeIkinParam.CoMTrajectoryError;
feetError(:,1)                  = visualizeIkinParam.feetError;         
momentumError(:,1)              = visualizeIkinParam.momentumError;      

%% Function to be integrated
integratedFunction              =  @(t,Chi) inverseKinematicsFunction(t,Chi,params);

%% Euler forward integrator (fixed step)
for kk = 2:dimTime

% state at step k
ChiIkin(:,kk)   = ChiIkin(:,kk-1) + tStep.*integratedFunction(t(kk-1),ChiIkin(:,kk-1));
 
% joint references and visualization parameters
[dChiIkin,visualizeIkinParam] = inverseKinematicsFunction(t(kk),ChiIkin(:,kk),params); 
 
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
    
