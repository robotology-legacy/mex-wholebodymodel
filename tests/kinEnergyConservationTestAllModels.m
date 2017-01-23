% main function for the kinematic conservation test,
% calls the actual test for all the models considered

fprintf('Running kinEnergyConservation\n');

% plot the kinematic energy ?
params.plot = false;

% raise error on fail ? Normally true, useful to turn it to false for debug
params.raiseErrorOnFail = true;

% tolerance for the relative energy error for the energy to be considered constant
params.relTol = 1e-3;

% params duration of simulation (in seconds)
params.simulationLengthInSecs = 10;

% models

% check full iCub model
params.yarpRobotName = 'icubGazeboSim';
params.isURDF = false;
kinEnergyConservationTest(params);

% check iCub from local urdf model
params.urdfFilePath = 'icub.urdf';
params.isURDF = true;
kinEnergyConservationTest(params);

% check iCub V2.5 no battery from local urdf model
params.urdfFilePath = 'iCubV2.5_nobattery.urdf';
params.isURDF = true;
kinEnergyConservationTest(params);

% check iCub V2.5 no battery from local urdf model
params.urdfFilePath = 'iCubV2.5.urdf';
params.isURDF = true;
kinEnergyConservationTest(params);

% check simple two links model
params.urdfFilePath = 'twoLinks.urdf';
params.isURDF = true;
kinEnergyConservationTest(params);

fprintf('kinEnergyConservation tests passed\n');
