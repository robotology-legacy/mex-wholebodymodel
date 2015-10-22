% main function for the kinematic conservation test,
% calls the actual test for all the models considered

% plot the kinematic energy ? 
params.plot = true;

% absolute tolerance (in joule) for the energy to be considered constant
params.absTolInJoule = 1e-2;

% params duration of simulation (in seconds)
params.simulationLengthInSecs = 1.0;

% models 

% check simple two links model
params.urdfFilePath = 'twoLinks.urdf';
params.isURDF = true;
kinEnergyConservationTest(params);

% check full iCub model 
params.yarpRobotName = 'icubGazeboSim';
params.isURDF = false;
kinEnergyConservationTest(params);

fprintf('kinEnergyConservation tests passed');