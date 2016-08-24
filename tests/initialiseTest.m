clear
close all;

fprintf('Running initialiseTest\n');

%% Try to load a model using the YARP_ROBOT_NAME before
wbm_modelInitialise('icubGazeboSim');

%% check if the limits has size 25
[min_iCub,max_iCub] = wbm_jointLimits();
WBMAssertEqual(size(min_iCub,1),25,'Error in size of iCub limits');

%% Now initialise mexWholeBodyModel given an urdf file
wbm_modelInitialiseFromURDF('twoLinks.urdf');

%% get limits
[min,max] = wbm_jointLimits();

%% for the twoLinks.urdf file, the limits should be -2.0 and 2.0
WBMAssertEqual(min,[-2],'Error in getting lower joint limits from urdf file');
WBMAssertEqual(max,[2],'Error in getting lower joint limits from urdf file');

fprintf('initialiseTest completed successfully\n');

