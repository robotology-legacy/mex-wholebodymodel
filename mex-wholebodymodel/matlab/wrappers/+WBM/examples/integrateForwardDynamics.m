% namespaces:
import WBM.*
import WBM.utilities.*
import WBM.Robot.iCub.*


%% Initialization of the WBM for the iCub-Robot:
wbm_iCub = initRobot_iCub();
iCub_model = wbm_iCub.robot_model;
iCub_config = wbm_iCub.robot_config;

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
fwdDynFunc = @(t, chi)WBM.utilities.fastForwardDynamics(t, chi, ctrlTrqs, wbm_iCub.robot_config);
%fwdDynFunc = @(t, chi)wbm_iCub.forwardDynamics(t, chi, ctrlTrqs); % optional

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

nRes = size(chi,1);
fprintf('Number of integration results: %d\n', nRes);

%% iCub-Simulator:

% setup the window and plot parameters for the WBM-simulator:
sim_config = initSimConfig_iCub();
sim_config = wbm_iCub.setupSimulation(sim_config);
x_out = wbm_iCub.getPositionsData(chi);
% show and repeat the simulation 10 times ...
nRpts = 6;
wbm_iCub.simulateForwardDynamics(x_out, sim_config, sim_time.step, nRpts);

%% Plot the results -- CoM-trajectory:
stPData = wbm_iCub.getStateParamsData(chi);

figure('Name', 'iCub - CoM-trajectory:', 'NumberTitle', 'off');

plot3(stPData.x_b(1:nRes,1), stPData.x_b(1:nRes,2), stPData.x_b(1:nRes,3), 'Color', 'b');
hold on;
plot3(stPData.x_b(1,1), stPData.x_b(1,2), stPData.x_b(1,3), 'Marker', 'o', 'MarkerEdgeColor', 'r');

grid on;
axis square;
xlabel('X(nRes)');
ylabel('Y(nRes)');
zlabel('Z(nRes)');
