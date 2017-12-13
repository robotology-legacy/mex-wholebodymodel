% namespaces:
import WBM.*
import WBM.utilities.ffun.*
import WBM.utilities.tfms.*
import WBM.RobotModel.iCub.*


%% Initialization of the WBM for the iCub robot:
wf2fixlnk   = true; % set the world frame to a fixed link
wbm_icub    = initRobotICub(wf2fixlnk);
icub_model  = wbm_icub.robot_model;
icub_config = wbm_icub.robot_config;

%% State variable:
%  Create the initial condition of the state variable "chi" for the integration of the
%  forward dynamics in state-space form. The state-space form reduces, through variable
%  substitution, the inhomogeneous second-order ODE to a first-order ODE.
%  For further details see:
%    [1] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008,
%        Chapter 3, pp. 40-42, formula (3.8).
chi_init = wbm_icub.init_stvChi;

%% Control torques:
%  Setup the time-dependent function "fhTrqControl" which describes a forcing function
%  on the ODEs to control the dynamics of the equation-system. It refers to the control
%  torques of each time-step t and is needed to calculate the constraint (contact) forces
%  which influences the outcome of each equation, the generalized acceleration dv (q_ddot).
[p_b, R_b] = frame2posRotm(wbm_icub.init_vqT_base);
g_init = wbm_icub.generalizedBiasForces(R_b, p_b, icub_config.init_state_params.q_j, ...
                                        zeros(icub_model.ndof,1), zeros(6,1));
len = size(g_init,1);

% Function handle (fh) for the torque controller function:
% Note: This function handle has only a very simple dummy-function as controller (zero
%       torques) that works in this case. For complex scenarios it is advisable to use
%       a real controller function instead, to avoid integration errors.
fhTrqControl = @(t)zeroTrqsController(size(g_init(7:len,1)));

%% ODE-Solver:
%  Setup the function handle of the form f(t,chi), where chi refers to the dynamic state
%  of the system. It evaluates the right side of the nonlinear first-order ODEs of the
%  form chi' = f(t,chi) and returns a vector of rates of change (vector of derivatives)
%  which will be integrated by the solver.
fhFwdDyn = @(t, chi)fastForwardDynamics(t, chi, fhTrqControl, ...
                                        wbm_icub.robot_model, wbm_icub.robot_config);
%fhFwdDyn = @(t, chi)wbm_icub.forwardDynamics(t, chi, fhTrqControl); % optional

% specifying the time interval of the integration ...
sim_time.start = 0.0;
sim_time.end   = 2.0; %1.5;
sim_time.step  = 0.01;
tspan = sim_time.start:sim_time.step:sim_time.end;

disp('Start the numerical integration...');

ode_options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);         % setup the error tolerances ...
[t, chi]    = ode15s(fhFwdDyn, tspan, chi_init, ode_options); % ODE-Solver
% or, optional:
%[t, chi] = fastIntForwardDynamics(fhTrqControl, tspan, chi_init, wbm_icub.robot_model, ...
%                                  wbm_icub.robot_config, ode_options);
%[t, chi] = wbm_icub.intForwardDynamics(tspan, chi_init, fhTrqControl, ode_options);

save('testTrajectory.mat', 't', 'chi', 'chi_init', 'fhTrqControl', 'icub_model', 'icub_config');
disp('Numerical integration finished.');

noi = size(chi,1);
fprintf('Number of integrations: %d\n', noi);

%% iCub-Simulator:

% Setup the window, the environment and the draw parameters for the WBM-simulator:

% create some geometric volume bodies for the simulation environment ...
rotm_r = eye(3,3); % rect. orientation
rotm_2 = [-0.9     0  -0.1;
           0    -0.9     0;
          -0.1     0   0.9];

vb_objects      = repmat(WBM.vbCuboid, 3, 1);
vb_objects(1,1) = WBM.vbCuboid(0.1, [0.15; 0.10; 0.61], rotm_r);
vb_objects(2,1) = WBM.vbCylinder(0.1, 0.2, [-0.2; 0.4; 0.4], rotm_2);
vb_objects(3,1) = WBM.vbSphere(0.1, [-0.3; 0.3; 0.2], rotm_r);

show_light = true;
sim_config = initSimConfigICub(vb_objects, show_light);             % shows the simulation with a light scene as default.
%sim_config = initSimConfigICub(vb_objects, 'DarkScn', show_light); % optional, shows the simulation with a dark scene.
sim_config = wbm_icub.setupSimulation(sim_config);

% define some payload links and link each payload link to a specified volume body:
% note: payload links are usually links of a manipulator (end-effector) or in
%       special cases, links on which additionally special payloads are mounted
%       (e.g. battery pack or knapsack at torso, tools, etc.).
pl_lnk_l.name     = 'l_gripper'; % --> l_hand_dh_frame --> l_hand
pl_lnk_l.lnk_p_cm = [0; 0; -0.05];
pl_lnk_l.vb_idx   = 1;
%pl_lnk_l.m_rb     = sim_config.environment.vb_objects(1,1).m_rb; % optional
%pl_lnk_l.I_cm     = sim_config.environment.vb_objects(1,1).I_cm;

wbm_icub.setPayloadLinks(pl_lnk_l);

% setup the payload stack to be processed:
% (link the index of the volume body object to the left hand (manipulator))
sim_config.setPayloadStack(pl_lnk_l.vb_idx, 'lh');
% set the utilization time indices (start, end) of the object:
sim_config.setPayloadUtilTime(1, 1, 35);
% optional:
%pl_lnk_r.name     = 'r_gripper'; % --> r_hand_dh_frame --> r_hand
%pl_lnk_r.lnk_p_cm = [0; 0; 0.05];
%pl_lnk_r.m_rb     = sim_config.environment.vb_objects(2,1).m_rb;
%pl_lnk_r.I_cm     = sim_config.environment.vb_objects(2,1).I_cm;
%pl_lnk_r.vb_idx   = 2;

%pl_lnk_data = {pl_lnk_l, pl_lnk_r};
%wbm_icub.setPayloadLinks(pl_lnk_data);

% link each vb-index of the objects to one hand (manipulator) ...
%sim_config.setPayloadStack([pl_lnk_l.vb_idx, pl_lnk_r.vb_idx], {'lh', 'rh'});
% utilization time indices of the payloads ...
%sim_config.setPayloadUtilTime(1, 1, 35);
%sim_config.setPayloadUtilTime(2, 1, 33);

% get the positions data of the integration output chi:
x_out = wbm_icub.getPositionsData(chi);

% show the trajectory curves of some specified links:
lnk_traj = repmat(WBM.wbmLinkTrajectory, 3, 1);
lnk_traj(1,1).urdf_link_name = 'l_gripper';
lnk_traj(2,1).urdf_link_name = 'r_gripper';
lnk_traj(3,1).urdf_link_name = 'r_lower_leg';

lnk_traj(1,1).jnt_annot_pos = {'left_arm', 7};
lnk_traj(2,1).jnt_annot_pos = {'right_arm', 7};
lnk_traj(3,1).jnt_annot_pos = {'right_leg', 4};

lnk_traj(1,1).line_color = WBM.wbmColor.forestgreen;
lnk_traj(1,1).ept_color  = WBM.wbmColor.forestgreen;
lnk_traj(2,1).line_color = WBM.wbmColor.tomato;
lnk_traj(2,1).ept_color  = WBM.wbmColor.tomato;
lnk_traj(3,1).line_color = 'magenta';
lnk_traj(3,1).ept_color  = 'magenta';

sim_config.trajectories = wbm_icub.setTrajectoriesData(lnk_traj, x_out, [1; 1; 1], [35; 35; 40]);
sim_config.show_legend  = true;

% zoom and shift some specified axes (optional):
%sim_config.zoomAxes([1 4], [0.9  1.1]);    % axes indices, zoom factors (90%, 110%)
%sim_config.shiftAxes(4, [0  0.05  -0.12]); %             , shift vectors (x, y, z)

% show and repeat the simulation 2 times ...
nRpts = 2;
wbm_icub.simulateForwardDynamics(x_out, sim_config, sim_time.step, nRpts);

%% Plot the results -- CoM-trajectory:
wbm_icub.plotCoMTrajectory(x_out);

% get the visualization data of the forward dynamics integration for plots and animations:
vis_data = wbm_icub.getFDynVisData(chi, fhTrqControl);

% alternatively, or if you have to plot other parameter values, use e.g.:
%stp = wbm_icub.getStateParams(chi);

%figure('Name', 'iCub - CoM-trajectory:', 'NumberTitle', 'off');

%plot3(stp.x_b(1:noi,1), stp.x_b(1:noi,2), stp.x_b(1:noi,3), 'Color', 'b');
%hold on;
%plot3(stp.x_b(1,1), stp.x_b(1,2), stp.x_b(1,3), 'Marker', 'o', 'MarkerEdgeColor', 'r');

%grid on;
%axis square;
%xlabel('x_{xb}');
%ylabel('y_{xb}');
%zlabel('z_{xb}');
