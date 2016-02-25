% namespaces:
import WBM.*
import WBM.utilities.*


%% Initialization of the WBM:
%
% base model parameters:
iCub_model = wbmBaseModelParams;
iCub_model.urdfRobot    = 'icubGazeboSim';
%iCub_model.urdfLinkName = 'l_sole';
iCub_model.wf_R_rootLnk = eye(3,3);
iCub_model.wf_p_rootLnk = zeros(3,1);
iCub_model.g_wf         = [0; 0; -9.81]; %zeros(6+iCub_config.ndof,1);
% base robot config:
iCub_config = wbmHumanoidConfig;
iCub_config.ndof          = 25;
iCub_config.nCstrs        = 2;
iCub_config.cstrLinkNames = {'l_sole', 'r_sole'};
iCub_config.dampCoeff     = 0.00; %0.75;
% body (joint) positions of the iCub-Robot (in degrees):
% (this configuration assumes an iCub-Robot with 25 DoFs.)
iCub_config.jpos_torso    = [-10.0; 0.0; 0.0];
iCub_config.jpos_leftArm  = [-19.7; 29.7; 0.0; 44.9; 0.0];
iCub_config.jpos_leftLeg  = [25.5; 0.1; 0.0; -38.5; -5.5; -0.1];
iCub_config.jpos_rightArm = iCub_config.jpos_leftArm;
iCub_config.jpos_rightLeg = iCub_config.jpos_leftLeg;
% init-state parameters:
iCub_config.initStateParams.x_b     = zeros(3,1);
iCub_config.initStateParams.qt_b    = zeros(4,1);
iCub_config.initStateParams.q_j     = [iCub_config.jpos_torso; iCub_config.jpos_leftArm; iCub_config.jpos_rightArm; ...
                                       iCub_config.jpos_leftLeg; iCub_config.jpos_rightLeg] * (pi/180.0); % in radians
iCub_config.initStateParams.dx_b    = zeros(3,1);
iCub_config.initStateParams.omega_b = zeros(3,1);
iCub_config.initStateParams.dq_j    = zeros(iCub_config.ndof,1);

% init the mex-WholeBodyModel for the iCub-Robot:
wf2FixLnk = true;
wbm_iCub = WBM(iCub_model, iCub_config, wf2FixLnk);

%% State variable:
%  Create the initial condition of the state variable "chi" for the integration of the
%  forward dynamics in state-space form. The state-space form reduces, through variable
%  substitution, the inhomogeneous second-order ODE to a first-order ODE.
%  For further details see:
%    "Rigid Body Dynamics Algorithms" of Roy Featherstone,
%    chapter 3, pages 40-42, formula (3.8).
chi_init = wbm_iCub.stvChiInit;

%% Control torques:
%  Setup the time-dependent variable "tau" which describes a forcing function/term on
%  the ODEs to control the dynamics of the equation-system. It refers to the control
%  torques of each time-step t and is needed to calculate the constraint forces f_c
%  which influences the outcome of each equation, the generalized acceleration dv (q_ddot).
vqT_init = wbm_iCub.vqTInit;
%vqT_init = chi_init(1:7,1);
[p_b, R_b] = frame2posRotm(vqT_init);
g_init = wbm_iCub.generalizedBiasForces(R_b, p_b, iCub_config.initStateParams.q_j, ...
                                        zeros(iCub_config.ndof,1), zeros(6,1));
len = size(g_init,1);

ctrlTrqs.tau = @(t)zeros(size(g_init(7:len)));

%% ODE-Solver:
%  Setup the function handle of the form f(t,chi) where chi refers to the dynamic state
%  of the system. It evaluates the right side of the nonlinear first-order ODEs of the
%  form chi' = f(t,chi) and returns a vector of rates of change (vector of derivatives)
%  that will be integrated by the solver.
wbm_config = wbm_iCub.wbm_config;
fwdDynFunc = @(t, chi)WBM.fastForwardDynamics(t, chi, ctrlTrqs, wbm_config);
%fwdDynFunc = @(t, chi)wbm_iCub.forwardDynamics(t, chi, ctrlTrqs);

% specifying the time interval of the integration ...
sim_time.start = 0.0;
sim_time.end   = 2.0; %1.5;
sim_time.step  = 0.01;
tspan = sim_time.start:sim_time.step:sim_time.end;

disp('Start the numerical integration:');

ode_options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);           % setup the error tolerances ...
[t, chi]    = ode15s(fwdDynFunc, tspan, chi_init, ode_options); % ODE-Solver

save('testTrajectory.mat', 't', 'chi', 'ctrlTrqs', 'iCub_model', 'iCub_config');
disp('Numerical integration finished.');

% compare the calculated data with the saved results of the original method ...
chi_orig = load('/../../matlab-src/storedTestTrajectory.mat', 'chi');
if isequal(chi, chi_orig.chi)
    disp('IDENTICAL --> CORRECT.');
else
    disp('NOT IDENTICAL --> NOT CORRECT.');
end

%% iCub-Simulator:
% it = 1;
% while (it < 10)
    % setup the window and plot parameters for the WBM-simulator:
    sim_config = genericSimConfig;
    sim_config.main_title = 'iCub-Simulator:';
    wbm_iCub.setupSimulation(sim_config);

    x_out = wbm_iCub.getPositionsData(chi);
    %wbm_iCub.visualizeForwardDynamics(x_out, t, sim_config); % not implemented yet ...
    %it = it + 1;
% end

%% Plot the results -- CoM-trajectory:
stPData = wbm_iCub.getStateParamsData(chi);
[m,~] = size(chi);

figure(2);

plot3(stPData.x_b(1:m,1), stPData.x_b(1:m,2), stPData.x_b(1:m,3), 'Color', 'b');
hold on;
plot3(stPData.x_b(1,1), stPData.x_b(1,2), stPData.x_b(1,3), 'Marker', 'o', 'MarkerEdgeColor', 'r');

grid on;
axis square;
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
