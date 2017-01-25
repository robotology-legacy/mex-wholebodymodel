function figureCont = visualizeSimulation_iDyntree(chi,CONFIG)
%VISUALIZESIMULATION_IDYNTREE calls iDyntree visualizer to simulate robot
%                             movements.
%
% figureCont = VISUALIZESIMULATION_IDYNTREE(chi,CONFIG) takes as input the
% robot state, chi, and the configuration parameters.
% The output is a counter for the automatic correction of figures numbers 
% in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, January 2017
%

% ------------Initialization----------------
%% initial parameters
figureCont       = CONFIG.figureCont;
mdlLdr           = CONFIG.mdlLdr;
consideredJoints = CONFIG.consideredJoints;

% get the joint position
CONFIG.demux.baseOrientationType = 1;
[~,qj,~,~] = stateDemux(chi',CONFIG);

% visualization time
init_time  = 1;
end_time   = length(qj(1,:));

% load the model from .urdf
mdlLdr.loadReducedModelFromFile('model/model.urdf',consideredJoints);
model = mdlLdr.model();

% open the visualizer
viz   = iDynTree.Visualizer();

viz.init();
viz.addModel(model,'icub');
viz.draw();

%% Robot simulation
for i=init_time:end_time

    tic
    jointPos = iDynTree.JointPosDoubleArray(model);
    joints   = qj(:,i);
    jointPos.fromMatlab(joints);

    % compute the world_H_base that correspond to the specified joints 
    odom = iDynTree.SimpleLeggedOdometry();
    odom.setModel(model);
    odom.updateKinematics(jointPos);
    
    if sum(CONFIG.feet_on_ground) == 2
    
        odom.init('l_sole','l_sole');
    
    elseif CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
    
        odom.init('l_sole','l_sole');
    
    elseif CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
    
        odom.init('r_sole','r_sole');
    end

    viz.modelViz(0).setPositions(odom.getWorldLinkTransform(model.getDefaultBaseLink()),jointPos);
    viz.draw();
    t = toc;
    pause(max(0,0.01-t))
end


