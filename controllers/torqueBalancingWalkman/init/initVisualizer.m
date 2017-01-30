function figureCont = initVisualizer(t,chi,CONFIG)
%INITVISUALIZER is the main function for visualizing the results of iCub forward
%               dynamics integration in MATLAB.
%
% INITVISUALIZER visualizes some outputs from the forward dynamics
% integration (e.g. the robot state, contact forces, control torques...).
%
% figureCont = INITVISUALIZER(t,chi,CONFIG) takes as input the integration
% time t, the robot state chi and the structure CONFIG containing all the
% utility parameters. The output is a counter for the automatic correction
% of figures numbers in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Configuration parameters
ndof                             = CONFIG.ndof;
initState                        = CONFIG.initState;
figureCont                       = CONFIG.figureCont;

%% Robot simulator
if CONFIG.visualize_robot_simulator == 1
    % list of joints used in the visualizer
    CONFIG.modelName        = 'Walkman';
    CONFIG.setPos           = [1.7,0,1];
    CONFIG.setCamera        = [0.4,0,1];
    CONFIG.mdlLdr           = iDynTree.ModelLoader();
    consideredJoints        = iDynTree.StringVector();
    
    consideredJoints.push_back('WaistSag');
    consideredJoints.push_back('WaistLat');
    consideredJoints.push_back('WaistYaw');
    consideredJoints.push_back('LShSag');
    consideredJoints.push_back('LShLat');
    consideredJoints.push_back('LShYaw');
    consideredJoints.push_back('LElbj');
    consideredJoints.push_back('LForearmPlate');
    consideredJoints.push_back('RShSag');
    consideredJoints.push_back('RShLat');
    consideredJoints.push_back('RShYaw');
    consideredJoints.push_back('RElbj');
    consideredJoints.push_back('RForearmPlate');
    consideredJoints.push_back('LHipSag');
    consideredJoints.push_back('LHipLat');
    consideredJoints.push_back('LHipYaw');
    consideredJoints.push_back('LKneeSag');
    consideredJoints.push_back('LAnkSag');
    consideredJoints.push_back('LAnkLat');
    consideredJoints.push_back('RHipSag');
    consideredJoints.push_back('RHipLat');
    consideredJoints.push_back('RHipYaw');
    consideredJoints.push_back('RKneeSag');
    consideredJoints.push_back('RAnkSag');
    consideredJoints.push_back('RAnkLat');

    % load the model from .urdf
    CONFIG.mdlLdr.loadReducedModelFromFile('../models/walkman/model.urdf',consideredJoints);
        
    % set lights
    CONFIG.lightDir = iDynTree.Direction();     
    CONFIG.lightDir.fromMatlab([0.5 0 0.5]/sqrt(2)); 
    
    visualizeSimulation_iDyntree(chi,CONFIG);
end

%% Forward dynamics results
if CONFIG.visualize_integration_results == 1  || CONFIG.visualize_joints_dynamics == 1
    
    CONFIG.wait = waitbar(0,'Generating the results...');
    set(0,'DefaultFigureWindowStyle','Docked');
    
    % joints initialization
    qj            = zeros(ndof,length(t));
    qjInit        = zeros(ndof,length(t));
    qjRef         = zeros(ndof,length(t));
    
    % contact forces and torques initialization
    fc            = zeros(6*CONFIG.numConstraints,length(t));
    f0            = zeros(6*CONFIG.numConstraints,length(t));
    tau           = zeros(ndof,length(t));
    tau_norm      = zeros(length(t),1);
    
    % forward kinematics initialization
    xCoM          = zeros(3,length(t));
    poseFeet      = zeros(12,length(t));
    CoP           = zeros(4,length(t));
    H             = zeros(6,length(t));
    HRef          = zeros(6,length(t));
    
    % generate the vectors from forward dynamics
    for time = 1:length(t)
        
        [~,visual]          = forwardDynamics(t(time), chi(time,:)', CONFIG);
        
        % joints dynamics
        qj(:,time)          = visual.qj;
        qjInit(:,time)      = initState.qj;
        qjRef(:,time)       = visual.jointRef.qjRef;
        
        %% Other parameters
        % contact forces and torques
        fc(:,time)          = visual.fc;
        f0(:,time)          = visual.f0;
        tau(:,time)         = visual.tau;
        tau_norm(time)      = norm(visual.tau);
        
        % forward kinematics
        xCoM(:,time)        = visual.xCoM;
        poseFeet(:,time)    = visual.poseFeet;
        H(:,time)           = visual.H;
        HRef(:,time)        = visual.HRef;
        
        % centers of pressure at feet
        CoP(1,time)         = -visual.fc(5)/visual.fc(3);
        CoP(2,time)         =  visual.fc(4)/visual.fc(3);
        
        if  CONFIG.numConstraints == 2
            
            CoP(3,time)     = -visual.fc(11)/visual.fc(9);
            CoP(4,time)     =  visual.fc(10)/visual.fc(9);
        end
        
    end
    
    delete(CONFIG.wait)
    HErr = H-HRef;
    
    %% Basic visualization (forward dynamics integration results)
    if CONFIG.visualize_integration_results == 1
        
        CONFIG.figureCont = visualizeForwardDyn(t,CONFIG,xCoM,poseFeet,fc,f0,tau_norm,CoP,HErr);
    end
    
    %% Joints positions and position error
    if CONFIG.visualize_joints_dynamics == 1
        
        CONFIG.figureCont = visualizeJointDynamics(t,CONFIG,qj,qjRef);
    end
    
    figureCont = CONFIG.figureCont;
    set(0,'DefaultFigureWindowStyle','Normal');
    
end
