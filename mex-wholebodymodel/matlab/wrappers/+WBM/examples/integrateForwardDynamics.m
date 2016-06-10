% namespaces:
import WBM.*
import WBM.utilities.*
import WBM.Robot.iCub.*


%% Initialization of the WBM for the iCub-Robot:
wf2fixLnk   = true; % set the world frame to a fixed link
wbm_icub    = initRobot_iCub(wf2fixLnk);
icub_model  = wbm_icub.robot_model;
icub_config = wbm_icub.robot_config;

%% State variable:
%  Create the initial condition of the state variable "chi" for the integration of the
%  forward dynamics in state-space form. The state-space form reduces, through variable
%  substitution, the inhomogeneous second-order ODE to a first-order ODE.
%  For further details see:
%    Rigid Body Dynamics Algorithms, Roy Featherstone, springer, 2008, chapter 3, pp. 40-42, formula (3.8).
chi_init = wbm_icub.stvChiInit;

%% Control torques:
%  Setup the time-dependent variable "tau" which describes a forcing function/term on
%  the ODEs to control the dynamics of the equation-system. It refers to the control
%  torques of each time-step t and is needed to calculate the constraint forces f_c
%  which influences the outcome of each equation, the generalized acceleration dv (q_ddot).
[p_b, R_b] = frame2posRotm(wbm_icub.vqTInit);
g_init = wbm_icub.generalizedBiasForces(R_b, p_b, icub_config.init_state_params.q_j, ...
                                        zeros(icub_model.ndof,1), zeros(6,1));
len = size(g_init,1);
% minimalistic data structure for the control torques (further parameters for
% different computations can be added to this structure):
ctrlTrqs.fhTau = @(t)zeros(size(g_init(7:len,1))); % torque function handle

%% ODE-Solver:
%  Setup the function handle of the form f(t,chi) where chi refers to the dynamic state
%  of the system. It evaluates the right side of the nonlinear first-order ODEs of the
%  form chi' = f(t,chi) and returns a vector of rates of change (vector of derivatives)
%  that will be integrated by the solver.
fhFwdDyn = @(t, chi)WBM.utilities.fastForwardDynamics(t, chi, ctrlTrqs.fhTau, ...
                                                      wbm_icub.robot_model, wbm_icub.robot_config);
%fhFwdDyn = @(t, chi)wbm_icub.forwardDynamics(t, chi, ctrlTrqs.fhTau); % optional

% specifying the time interval of the integration ...
sim_time.start = 0.0;
sim_time.end   = 2.0; %1.5;
sim_time.step  = 0.01;
tspan = sim_time.start:sim_time.step:sim_time.end;

disp('Start the numerical integration...');

ode_options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);         % setup the error tolerances ...
[t, chi]    = ode15s(fhFwdDyn, tspan, chi_init, ode_options); % ODE-Solver

save('testTrajectory.mat', 't', 'chi', 'chi_init', 'ctrlTrqs', 'icub_model', 'icub_config');
disp('Numerical integration finished.');

nSteps = size(chi,1);
fprintf('Number of integrations: %d\n', nSteps);

%% iCub-Simulator:

% setup the window and plot parameters for the WBM-simulator:
sim_config = initSimConfig_iCub();           % shows the simulation with a light scene as default ...
%sim_config = initSimConfig_iCub('DarkScn'); % optional, shows the simulation with a dark scene ...
sim_config = wbm_icub.setupSimulation(sim_config);
x_out = wbm_icub.getPositionsData(chi);
% show and repeat the simulation 10 times ...
nRpts = 5;
wbm_icub.simulateForwardDynamics(x_out, sim_config, sim_time.step, nRpts);

%% Plot the results -- CoM-trajectory:
wbm_icub.plotCoMTrajectory(chi);

% alternative, or if you have to plot other parameter values, use e.g.:
%stp = wbm_icub.getStateParams(chi);

%figure('Name', 'iCub - CoM-trajectory:', 'NumberTitle', 'off');

%plot3(stp.x_b(1:nSteps,1), stp.x_b(1:nSteps,2), stp.x_b(1:nSteps,3), 'Color', 'b');
%hold on;
%plot3(stp.x_b(1,1), stp.x_b(1,2), stp.x_b(1,3), 'Marker', 'o', 'MarkerEdgeColor', 'r');

%grid on;
%axis square;
%xlabel('x_{xb}');
%ylabel('y_{xb}');
%zlabel('z_{xb}');
