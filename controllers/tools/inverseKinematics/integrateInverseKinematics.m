function REFERENCES = integrateInverseKinematics(MODEL,chiInit)
%INTEGRATEINVERSEKINEMATICS integrates the inverse kinematics using a fixed
%                           step Eulero forward integrator.
%
% Format:  REFERENCES = INTEGRATEINVERSEKINEMATICS(MODEL,chiInit)
%
% Inputs:  - MODEL: it is a structure defining the robot model;        
%          - chInit: robot state vector [13+2*ndof x 1];
%
% Output:  - REFERENCES: it is a structure containing the joints reference 
%                        trajectory, velocity and acceleration. 
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% setup integration time and initial conditions
ndof                = MODEL.ndof;
tStep               = MODEL.CONFIG.sim_step;
t                   = transpose(MODEL.CONFIG.tStart:tStep:MODEL.CONFIG.tEnd);
dimTime             = length(t);
dimState            = length(chiInit);
% initial conditions for inverse dynamics integration
qj                  = zeros(ndof,dimTime);
dqj                 = qj;
ddqj                = dqj;
chi                 = zeros(dimState,dimTime);
chi(:,1)            = chiInit;
qj(:,1)             = chiInit(8:7+ndof);
dqj(:,1)            = chiInit(14+ndof:end);
% function to be integrated
integratedFunction  = @(t,chi) inverseKinematics(t,chi,MODEL);

%% Visualize inverse kinematics integration results
if MODEL.CONFIG.enable_visualTool
    % initialize all variables (the advantage of this method is the
    % variables are preallocated)
    ndof                 = MODEL.ndof;
    feetError            = zeros(12,dimTime); %#ok<NASGU>
    CoMError             = zeros(9,dimTime);  %#ok<NASGU>
    CoMRef               = zeros(3,dimTime);  %#ok<NASGU>
    HError               = zeros(6,dimTime);  %#ok<NASGU>
    % create a folder and save stored values inside it
    outputDir = './media';
    if (~exist(outputDir, 'dir'))
        mkdir(outputDir);
    end
    % save the data in a .mat file
    save('./media/storedValues','feetError','CoMError','CoMRef','HError','-v7.3');
    % call the inverse kinematics funtion at tStart, for initializing the
    % stored values
    dchiInit             = inverseKinematics(t(1),chiInit,MODEL);
end
% initialize the joint reference accelerations
ddqj(:,1)                = dchiInit(14+ndof:end);

%% %%%%%%%%%%%%%%%%%%%%% FIXED STEP INTEGRATION %%%%%%%%%%%%%%%%%%%%%%%% %%
for kk = 2:dimTime
    % calculate the robot state at step kk
    chi(:,kk)   = chi(:,kk-1) + tStep.*integratedFunction(t(kk-1),chi(:,kk-1));
    % state derivative at step kk
    dchi        = inverseKinematics(t(kk),chi(:,kk),MODEL);
    % joint references at step kk
    qj(:,kk)    = chi(8:7+ndof,kk);
    dqj(:,kk)   = chi(14+ndof:end,kk);
    ddqj(:,kk)  = dchi(14+ndof:end);
end

%% Store joint reference trajectory, robot state and integration time
REFERENCES.chi_ikin  = chi;
REFERENCES.qj        = qj;
REFERENCES.dqj       = dqj;
REFERENCES.ddqj      = ddqj;
REFERENCES.t         = t;

end

