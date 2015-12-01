% namespaces:
import WBM.*
import WBM.utilities.*


%% First initialization of the WBM:
% base model:
iCub_model = wbmBaseModelParams;
iCub_model.urdfRobot = 'icubGazeboSim';
iCub_model.wf_R_rootLnk = eye(3,3);
iCub_model.g_wf         = [0; 0; 9.81];
% base robot config:
iCub_config = wbmHumanoidConfig;
iCub_config.ndof          = int16(25); 
iCub_config.nCstrs        = int16(2); 
iCub_config.cstrLinkNames = {'r_sole', 'l_gripper'};

nIter    = 1000;
initTime = 0;
totTime  = 0;

fprintf('Starting normal mode trial\n--------------------------\n');

tic;
wbm_iCub = WBM(iCub_model, iCub_config);
initTime = toc();

fprintf('Initialisation time : %e secs\n', initTime);
fprintf('Num Trials : %d\nStarting Trial...\n', nIter);

R = iCub_model.wf_R_rootLnk;
g = iCub_model.g_wf;
p = zeros(3,1);

q_j  = zeros(25,1);
dq_j = zeros(25,1);
v_b  = zeros(6,1);

tic;
for i = 1:nIter
    % set the current state with random values ...
    q_j  = rand(25,1);
    dq_j = rand(25,1);
    v_b  = rand(6,1);
    % update the translation to the WF with random values ...`
    p    = rand(3,1);

    wbm_iCub.setWorldFrame(R, p, g);
    
    % calculate some mex-WholeBodyModel (WBM) Components:
    M      = wbm_iCub.massMatrix(R, p, q_j);

    h_c    = wbm_iCub.generalBiasForces(R, p, q_j, dq_j, v_b);    
    h_c    = wbm_iCub.generalBiasForces(R, p, q_j, dq_j, v_b); 
    h_c    = wbm_iCub.generalBiasForces(R, p, q_j, dq_j, v_b);

    dJdq_1 = wbm_iCub.dJdq(iCub_config.cstrLinkNames{1}, R, p, q_j, dq_j, v_b);
    dJdq_2 = wbm_iCub.dJdq(iCub_config.cstrLinkNames{2}, R, p, q_j, dq_j, v_b);

    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{1}, R, p, q_j);
    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{2}, R, p, q_j);
    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{1}, R, p, q_j);
    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{2}, R, p, q_j);
end
totTime = toc();

%% Output the first time-benchmark results:
fprintf('Normal-Mode Trial Total Time : %f secs\n', totTime);
fprintf('Normal-Mode Trial Average Time : %e secs\n', totTime/nIter);

clear all; % clear completely the workspace ...

fprintf('\n\nStarting optimised mode trial\n--------------------------\n');

%% Second initialization of the WBM:
% base model:
iCub_model = wbmBaseModelParams;
iCub_model.urdfRobot = 'icubGazeboSim';
iCub_model.wf_R_rootLnk = eye(3,3);
iCub_model.g_wf         = [0; 0; 9.81];
% base robot config:
iCub_config = wbmHumanoidConfig;
iCub_config.ndof          = int16(25); 
iCub_config.nCstrs        = int16(2);
iCub_config.cstrLinkNames = {'r_sole', 'l_gripper'};

nIter    = 1000;
initTime = 0;
totTime  = 0;

tic;
wbm_iCub = WBM(iCub_model, iCub_config);
initTime = toc();

fprintf('Initialisation time : %e secs\nStarting Trial...\n ', initTime);

R = iCub_model.wf_R_rootLnk;
g = iCub_model.g_wf;
p = zeros(3,1);

q_j  = zeros(25,1);
dq_j = zeros(25,1);
v_b  = zeros(6,1);

tic;
for i = 1:nIter 
    % fill the state with random values ...
    q_j  = rand(25,1);
    dq_j = rand(25,1);
    v_b  = rand(6,1);
    % fill the translation to the WF with random values ...
    p    = rand(3,1);

    % wbm_iCub.setState(q_j, dq_j, v_b);
    wbm_iCub.setWorldFrame(R, p, g);
    wbm_iCub.setState(q_j, dq_j, v_b);
    
    % mex-WholeBodyModel functions ...
    M      = wbm_iCub.massMatrix();    
    
    h_c    = wbm_iCub.generalBiasForces();    
    h_c    = wbm_iCub.generalBiasForces();  
    h_c    = wbm_iCub.generalBiasForces();  

    dJdq_1 = wbm_iCub.dJdq(iCub_config.cstrLinkNames{1});
    dJdq_2 = wbm_iCub.dJdq(iCub_config.cstrLinkNames{2});

    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{1});
    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{2});
    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{1});
    J      = wbm_iCub.jacobian(iCub_config.cstrLinkNames{2});
end
totTime = toc();

%% Output the second time-benchmarks:
fprintf('Optimised-Mode Trial Total Time : %f secs\n', totTime);
fprintf('Optimised-Mode Trial Average Time : %e secs\n', totTime/nIter);
