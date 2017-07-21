% namespaces:
import WBM.*


%% First initialization of the WBM:
% base model:
icub_model = wbmBaseRobotModel;
icub_model.ndof            = 25;
icub_model.urdf_robot_name = 'icubGazeboSim';
icub_model.wf_R_b_init     = eye(3,3);
icub_model.g_wf            = [0; 0; -9.81];
% base robot config:
icub_config = wbmHumanoidConfig;
icub_config.nCstrs           = 2;
icub_config.ccstr_link_names = {'r_sole', 'l_gripper'};

noi = 1000;

fprintf('Starting normal mode trial\n--------------------------\n');

tic;
wbm_icub = WBM(icub_model, icub_config);
initTime = toc();

fprintf('Initialization time: %e secs\n', initTime);
fprintf('Num of Trials: %d\nStarting Trial...\n', noi);

R = icub_model.wf_R_b_init;
g = icub_model.g_wf;

tic;
for i = 1:noi
    % set the current state with random values ...
    q_j  = rand(25,1);
    dq_j = rand(25,1);
    v_b  = rand(6,1);
    % set the translation to the WF with some random values ...
    p    = rand(3,1);

    wbm_icub.setWorldFrame(R, p, g);

    % calculate some mex-WholeBodyModel (WBM) components:
    M      = wbm_icub.massMatrix(R, p, q_j);

    h_c    = wbm_icub.generalizedBiasForces(R, p, q_j, dq_j, v_b);
    h_c    = wbm_icub.generalizedBiasForces(R, p, q_j, dq_j, v_b);
    h_c    = wbm_icub.generalizedBiasForces(R, p, q_j, dq_j, v_b);

    c_qv   = wbm_icub.coriolisBiasForces(R, p, q_j, dq_j, v_b);
    c_qv   = wbm_icub.coriolisBiasForces(R, p, q_j, dq_j, v_b);

    g_q    = wbm_icub.gravityBiasForces(R, p, q_j);
    g_q    = wbm_icub.gravityBiasForces(R, p, q_j);

    djdq_1 = wbm_icub.dJdq(R, p, q_j, dq_j, v_b, icub_config.ccstr_link_names{1});
    djdq_2 = wbm_icub.dJdq(R, p, q_j, dq_j, v_b, icub_config.ccstr_link_names{2});

    J      = wbm_icub.jacobian(R, p, q_j, icub_config.ccstr_link_names{1});
    J      = wbm_icub.jacobian(R, p, q_j, icub_config.ccstr_link_names{2});

    J      = wbm_icub.jacobian(R, p, q_j, icub_config.ccstr_link_names{1});
    J      = wbm_icub.jacobian(R, p, q_j, icub_config.ccstr_link_names{2});
end
totTime = toc();

%% Output the first time-benchmark result:
fprintf('Normal-Mode Trial Total Time: %f secs\n', totTime);
fprintf('Normal-Mode Trial Average Time: %e secs\n', totTime/noi);

clearvars; % clear all variables from the workspace ...

%% Second initialization of the WBM:
fprintf('\n\nStarting optimized mode trial\n-----------------------------\n');

% base model:
icub_model = wbmBaseRobotModel;
icub_model.ndof            = 25;
icub_model.urdf_robot_name = 'icubGazeboSim';
icub_model.wf_R_b_init     = eye(3,3);
icub_model.g_wf            = [0; 0; 9.81];
% base robot config:
icub_config = wbmHumanoidConfig;
icub_config.nCstrs           = 2;
icub_config.ccstr_link_names = {'r_sole', 'l_gripper'};

noi = 1000;

tic;
wbm_icub = WBM(icub_model, icub_config);
initTime = toc();

fprintf('Initialization time: %e secs\nStarting Trial...\n', initTime);

R = icub_model.wf_R_b_init;
g = icub_model.g_wf;

tic;
for i = 1:noi
    % fill the state with some random values ...
    q_j  = rand(25,1);
    dq_j = rand(25,1);
    v_b  = rand(6,1);
    % fill the translation to the WF with random values ...
    p    = rand(3,1);

    wbm_icub.setState(q_j, dq_j, v_b);
    wbm_icub.setWorldFrame(R, p, g);

    % call some mex-WBM functions ...
    M      = wbm_icub.massMatrix();

    h_c    = wbm_icub.generalizedBiasForces();
    h_c    = wbm_icub.generalizedBiasForces();
    h_c    = wbm_icub.generalizedBiasForces();

    c_qv   = wbm_icub.coriolisBiasForces();
    c_qv   = wbm_icub.coriolisBiasForces();

    g_q    = wbm_icub.gravityBiasForces();
    g_q    = wbm_icub.gravityBiasForces();

    djdq_1 = wbm_icub.dJdq(icub_config.ccstr_link_names{1});
    djdq_2 = wbm_icub.dJdq(icub_config.ccstr_link_names{2});

    J      = wbm_icub.jacobian(icub_config.ccstr_link_names{1});
    J      = wbm_icub.jacobian(icub_config.ccstr_link_names{2});

    J      = wbm_icub.jacobian(icub_config.ccstr_link_names{1});
    J      = wbm_icub.jacobian(icub_config.ccstr_link_names{2});
end
totTime = toc();

%% Output the result of the second time-benchmark:
fprintf('Optimized-Mode Trial Total Time: %f secs\n', totTime);
fprintf('Optimized-Mode Trial Average Time: %e secs\n', totTime/noi);
