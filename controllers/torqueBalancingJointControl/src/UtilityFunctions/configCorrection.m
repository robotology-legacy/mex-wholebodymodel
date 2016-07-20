function [correctedCONFIG] = configCorrection(CONFIG)
%CONFIGCORRECTION corrects the user-defined initial configuration of the
%                 robot to avoid unfeasible configurations.
%
% [correctedCONFIG] = configCorrection(CONFIG) takes as input the structure
% CONFIG containing all the configuration parameters. The output is the
% same structure, but with all the possible configuration conficts
% resolved. E.g., is the user sets the balancing controller to be 'joint
% space', then all the features related to linearization and QP solver
% will be disabled here. 
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Initialize the output
correctedCONFIG  = CONFIG;

%% Forward dynamics integration correction
% if one wants to integrate the forward dynamics using the fixed step
% integrator, the mass matrix must be corrected to avoid singularities
if correctedCONFIG.integrateWithFixedStep == 1
    
correctedCONFIG.massCorr = 0.05; 
end

%% Postural task correction
% if one wants to use the postural correction, it is necessary for now to 
% define the joint references using inverse kinematics
if correctedCONFIG.postCorrection == 1
    
correctedCONFIG.jointRef_with_ikin  = 1;
end

%% Visualization setup
% this script modifies the default MATLAB options for figures and graphics
plot_set

% this is the figure counter. It is used to automatically adapt the figure
% number in case new figures are added
correctedCONFIG.figureCont          = 1;

%% Joint space controller
% the gain tuning procedure and the QP solver are not available for joint
% space controller; furthermore, it is necessary to compute the joint
% reference trajectory using inverse kinematics
if strcmp(correctedCONFIG.controller,'JointSpace') == 1
    
correctedCONFIG.gains_tuning                        = 0;                          
correctedCONFIG.use_QPsolver                        = 0;
correctedCONFIG.jointRef_with_ikin                  = 1;
end

%% Gains tuning
% since the gain tuning is based on the joint reference trajectory, the
% inverse kinematics is required
if correctedCONFIG.gains_tuning  == 1
    
correctedCONFIG.jointRef_with_ikin  = 1;
end

%% LINEARIZATION DEBUG AND STABILITY ANALYSIS MODE
% enter in debug mode
if correctedCONFIG.linearizationDebug == 1
    
correctedCONFIG.demo_movements                        = 0;                          
correctedCONFIG.controller                            = 'StackOfTask';               
correctedCONFIG.use_QPsolver                          = 0;                                       
correctedCONFIG.gains_tuning                          = 0;                     
end

end

