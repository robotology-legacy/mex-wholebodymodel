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

%% Robot simulator
if CONFIG.visualize_robot_simulator == 1
    % list of joints used in the visualizer
    CONFIG.modelName        = 'Walkman';
    CONFIG.setCamera        = [2,0,0.2];
    CONFIG.mdlLdr           = iDynTree.ModelLoader();
    CONFIG.consideredJoints = iDynTree.StringVector();
    
    CONFIG.consideredJoints.push_back('WaistSag');
    CONFIG.consideredJoints.push_back('WaistLat');
    CONFIG.consideredJoints.push_back('WaistYaw');
    CONFIG.consideredJoints.push_back('LShSag');
    CONFIG.consideredJoints.push_back('LShLat');
    CONFIG.consideredJoints.push_back('LShYaw');
    CONFIG.consideredJoints.push_back('LElbj');
    CONFIG.consideredJoints.push_back('LForearmPlate');
    CONFIG.consideredJoints.push_back('RShSag');
    CONFIG.consideredJoints.push_back('RShLat');
    CONFIG.consideredJoints.push_back('RShYaw');
    CONFIG.consideredJoints.push_back('RElbj');
    CONFIG.consideredJoints.push_back('RForearmPlate');
    CONFIG.consideredJoints.push_back('LHipSag');
    CONFIG.consideredJoints.push_back('LHipLat');
    CONFIG.consideredJoints.push_back('LHipYaw');
    CONFIG.consideredJoints.push_back('LKneeSag');
    CONFIG.consideredJoints.push_back('LAnkSag');
    CONFIG.consideredJoints.push_back('LAnkLat');
    CONFIG.consideredJoints.push_back('RHipSag');
    CONFIG.consideredJoints.push_back('RHipLat');
    CONFIG.consideredJoints.push_back('RHipYaw');
    CONFIG.consideredJoints.push_back('RKneeSag');
    CONFIG.consideredJoints.push_back('RAnkSag');
    CONFIG.consideredJoints.push_back('RAnkLat');
    
    CONFIG.figureCont = visualizeSimulation_iDyntree(chi,CONFIG);
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
