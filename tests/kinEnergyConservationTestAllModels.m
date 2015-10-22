% main function for the kinematic conservation test,
% calls the actual test for all the models considered

fprintf('Running kinEnergyConservation\n');

% plot the kinematic energy ?
params.plot = true;

% exit on failure ?
params.exitOnFailure = false;

% tolerance for the relative energy error for the energy to be considered constant
params.relTol = 1e-3;

% params duration of simulation (in seconds)
params.simulationLengthInSecs = 1.0;

% models

% check full iCub model
params.yarpRobotName = 'icubGazeboSim';
params.isURDF = false;
kinEnergyConservationTest(params);

% check iCub from local urdf model
params.urdfFilePath = 'icub.urdf';
params.isURDF = true;
kinEnergyConservationTest(params);

% check full iCub model again
params.yarpRobotName = 'icubGazeboSim';
params.isURDF = false;
kinEnergyConservationTest(params);

% check simple two links model
params.urdfFilePath = 'twoLinks.urdf';
params.isURDF = true;
kinEnergyConservationTest(params);

fprintf('kinEnergyConservation tests passed\n');