% namespaces:
import WBM.*
import WBM.utilities.*


%% First initialization of the WBM:
% base model:
iCub_model = wbmBaseModelParams;
iCub_model.urdfRobot = 'icubGazeboSim';
iCub_model.wf_R_rootLnk = eye(3,3);
iCub_model.g_wf         = [0; 0; -9.81];
% base robot config:
iCub_config = wbmHumanoidConfig;
iCub_config.ndof          = 25;
iCub_config.nCstrs        = 2;
iCub_config.cstrLinkNames = {'r_sole', 'l_gripper'};

noi = 1000;

fprintf('Starting normal mode trial\n--------------------------\n');

tic;
wbm_iCub = WBM(iCub_model, iCub_config);
initTime = toc();

fprintf('Initialization time : %e secs\n', initTime);
fprintf('Num Trials : %d\nStarting Trial...\n', noi);

R = iCub_model.wf_R_rootLnk;
g = iCub_model.g_wf;

tic;
for i = 1:noi
    % set the current state with random values ...
    q_j  = rand(25,1);
    dq_j = rand(25,1);
    v_b  = rand(6,1);
    % set the translation to the WF with some random values ...
    p    = rand(3,1);

    wbm_iCub.setWorldFrame(R, p, g);

    % calculate some mex-WholeBodyModel (WBM) components:
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

%% Output the first time-benchmark result:
fprintf('Normal-Mode Trial Total Time : %f secs\n', totTime);
fprintf('Normal-Mode Trial Average Time : %e secs\n', totTime/noi);

clearvars; % clear all variables from the workspace ...

%% Second initialization of the WBM:
fprintf('\n\nStarting optimized mode trial\n-----------------------------\n');

% base model:
iCub_model = wbmBaseModelParams;
iCub_model.urdfRobot = 'icubGazeboSim';
iCub_model.wf_R_rootLnk = eye(3,3);
iCub_model.g_wf         = [0; 0; 9.81];
% base robot config:
iCub_config = wbmHumanoidConfig;
iCub_config.ndof          = 25;
iCub_config.nCstrs        = 2;
iCub_config.cstrLinkNames = {'r_sole', 'l_gripper'};

noi = 1000;

tic;
wbm_iCub = WBM(iCub_model, iCub_config);
initTime = toc();

fprintf('Initialization time : %e secs\nStarting Trial...\n', initTime);

R = iCub_model.wf_R_rootLnk;
g = iCub_model.g_wf;

tic;
for i = 1:noi
    % fill the state with some random values ...
    q_j  = rand(25,1);
    dq_j = rand(25,1);
    v_b  = rand(6,1);
    % fill the translation to the WF with random values ...
    p    = rand(3,1);

    wbm_iCub.setState(q_j, dq_j, v_b);
    wbm_iCub.setWorldFrame(R, p, g);

    % call some mex-WBM functions ...
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

%% Output the result of the second time-benchmark:
fprintf('Optimized-Mode Trial Total Time : %f secs\n', totTime);
fprintf('Optimized-Mode Trial Average Time : %e secs\n', totTime/noi);
