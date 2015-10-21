clear
close all;

%% initialise mexWholeBodyModel given an urdf file
wbm_modelInitialiseFromURDF('twoLinks.urdf');

%% get limits
[min,max] = wbm_jointLimits();

%% for the twoLinks.urdf file, the limits should be -2.0 and 2.0
WBMAssertEqual(min,[-2],'Error in getting lower joint limits from urdf file');
WBMAssertEqual(max,[2],'Error in getting lower joint limits from urdf file');

