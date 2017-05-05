function VISUALIZER = configureSimulator()
%CONFIGURESIMULATOR configures iDyntree simulator.
%
% Format: VISUALIZER = CONFIGURESIMULATOR(chi,CONFIG)
%          
% Output:  - VISUALIZER a structure containing the initialization parameters 
%            for visualization.  
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% List of joints used in the visualizer
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
model                       = VISUALIZER.mdlLdr.model();
% load the model from .urdf
VISUALIZER.mdlLdr.loadReducedModelFromFile('../../models/icub/model.urdf',consideredJoints);        
% set lights
VISUALIZER.lightDir = iDynTree.Direction();     
VISUALIZER.lightDir.fromMatlab([-0.5 0 -0.5]/sqrt(2)); 
% open the visualizer
VISUALIZER.viz      = iDynTree.Visualizer();
VISUALIZER.viz.init();
VISUALIZER.viz.addModel(model,VISUALIZER.modelName);

end