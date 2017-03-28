function VISUALIZER = configureRobotSimulator(chi,CONFIG)
%CONFIGUREROBOTSIMULATOR configures iDyntree simulator.
%
% VISUALIZER = CONFIGUREROBOTSIMULATOR(chi,CONFIG) takes as an input the 
% current state of the robot chi and the configuration parameters. The
% output is the structure VISUALIZER containing all parameter for
% initializing iDyntree visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

% ------------Initialization----------------
%% List of joints used in the visualizer
consideredJoints = iDynTree.StringVector();
consideredJoints.push_back('torso_pitch');
consideredJoints.push_back('torso_roll');
consideredJoints.push_back('torso_yaw');
consideredJoints.push_back('l_shoulder_pitch');
consideredJoints.push_back('l_shoulder_roll');
consideredJoints.push_back('l_shoulder_yaw');
consideredJoints.push_back('l_elbow');
consideredJoints.push_back('l_wrist_prosup');
consideredJoints.push_back('r_shoulder_pitch');
consideredJoints.push_back('r_shoulder_roll');
consideredJoints.push_back('r_shoulder_yaw');
consideredJoints.push_back('r_elbow');
consideredJoints.push_back('r_wrist_prosup');
consideredJoints.push_back('l_hip_pitch');
consideredJoints.push_back('l_hip_roll');
consideredJoints.push_back('l_hip_yaw');
consideredJoints.push_back('l_knee');
consideredJoints.push_back('l_ankle_pitch');
consideredJoints.push_back('l_ankle_roll');
consideredJoints.push_back('r_hip_pitch');
consideredJoints.push_back('r_hip_roll');
consideredJoints.push_back('r_hip_yaw');
consideredJoints.push_back('r_knee');
consideredJoints.push_back('r_ankle_pitch');
consideredJoints.push_back('r_ankle_roll');

%% Robot and camera configuration
VISUALIZER.modelName        = 'iCub';
VISUALIZER.setPos           = [1,0,0.5];    
VISUALIZER.setCamera        = [0.4,0,0.5];
VISUALIZER.mdlLdr           = iDynTree.ModelLoader();
VISUALIZER.model            = VISUALIZER.mdlLdr.model();
% get the joint position
CONFIG.demux.baseOrientationType = 1;
[~,qj,~,~]       = stateDemux(chi,CONFIG);
% load the model from .urdf
VISUALIZER.mdlLdr.loadReducedModelFromFile('../models/icub/model.urdf',consideredJoints);
% set lights
VISUALIZER.lightDir = iDynTree.Direction();     
VISUALIZER.lightDir.fromMatlab([-0.5 0 -0.5]/sqrt(2)); 

%% Open the visualizer
VISUALIZER.viz  = iDynTree.Visualizer();
VISUALIZER.viz.init();
VISUALIZER.viz.addModel(VISUALIZER.model,VISUALIZER.modelName);
VISUALIZER.viz.draw();

%% Setup environment and lights     
% disable environmental features     
VISUALIZER.env = VISUALIZER.viz.enviroment();        
VISUALIZER.env.setElementVisibility('root_frame',false);   
% set lights
VISUALIZER.sun = VISUALIZER.viz.enviroment().lightViz('sun');         
VISUALIZER.sun.setDirection(VISUALIZER.lightDir);
% set camera     
VISUALIZER.cam = VISUALIZER.viz.camera();     
VISUALIZER.cam.setPosition(iDynTree.Position(VISUALIZER.setPos(1),VISUALIZER.setPos(2),VISUALIZER.setPos(3)));     
VISUALIZER.cam.setTarget(iDynTree.Position(VISUALIZER.setCamera(1),VISUALIZER.setCamera(2),VISUALIZER.setCamera(3))); 
    
%% Robot simulation
VISUALIZER.jointPos = iDynTree.JointPosDoubleArray(VISUALIZER.model);
VISUALIZER.jointPos.fromMatlab(qj);
% compute the world_H_base that correspond to the specified joints 
VISUALIZER.odom = iDynTree.SimpleLeggedOdometry();
VISUALIZER.odom.setModel(VISUALIZER.model);
VISUALIZER.odom.updateKinematics(VISUALIZER.jointPos);   
if sum(CONFIG.feet_on_ground) == 2
    
    VISUALIZER.odom.init('l_sole','l_sole');
    
elseif CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
    
    VISUALIZER.odom.init('l_sole','l_sole');
    
elseif CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
    
    VISUALIZER.odom.init('r_sole','r_sole');
end    
VISUALIZER.viz.modelViz(0).setPositions(VISUALIZER.odom.getWorldLinkTransform(VISUALIZER.model.getDefaultBaseLink()),VISUALIZER.jointPos);
VISUALIZER.viz.draw();

end