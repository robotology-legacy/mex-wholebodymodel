Introduction
============

The *Whole-Body Model Library* (WBML) is an open source library for Matlab and provides, outside of
of Simulink, fast *dynamics computation methods* for prototyping and testing purposes. The methods
of the library are mainly based on the open source C++ program ``mexWholeBodyModel``, which is a
Matlab executable subroutine (MEX) and provides for Matlab a high-level interface to the
``yarpWholeBodyInterface`` for YARP-based robots, such as the iCub humanoid robot.

The ``yarpWholeBodyInterface`` [#f1]_ is a fast C++ interface to a *Whole-Body Abstraction Layer*
for floating-base robots with YARP that contains subinterfaces for the *kinematic/dynamic model*,
*actuators*, *sensor measurements* and *state estimations*.

The WBM-Library is an object-oriented extension for the Matlab MEX whole-body model interface, which
was initially developed and supported by the FP7 EU-project *CoDyCo*
(`www.codyco.eu <http://www.codyco.eu>`_) at the *Istituto Italiano di Tecnologia* (IIT) in Genoa in
Italy.

Currently, the documentation covers only the most important classes of the WBM-Library. The *utility
functions* and the *interface classes* have not yet been documented in the library. The library
documentation will be gradually extended over time. The function names of the utility functions are
mostly oriented on the function names of the *Coordinate System Transformations* [#f2]_ of the
*Robotics System Toolbox* of Matlab.

The additional interface classes of the WBM-Library ensures the ease of integration in other
frameworks or research projects. The library offers two implemented interface classes:

   - ``iCubWBM`` -- An interface for the *iCub humanoid robot* for general purposes.
   - ``MultChainTree`` -- A special adapted interface for the *Robotics Toolbox* of *Peter Corke*
     (`www.petercorke.com <http://www.petercorke.com>`_). With this special interface, the iCub
     robot can be easily integrated into research projects that are using for their experiments
     fixed-base robot models of the toolbox. The method names and input arguments of the interface
     class are almost the same as the methods of the ``SerialLink`` class of the Robotics Toolbox.

.. [#f1] See `<https://github.com/robotology/yarp-wholebodyinterface>`_.
.. [#f2] See `<https://www.mathworks.com/help/robotics/coordinate-system-transformations.html>`_.

Example
-------

The given example in :numref:`code_example_int_fd` shows the basic usage of the WBM-Library. The
robot model will be loaded from the given URDF description into the ``yarpWholeBodyInterface`` that
is embedded in the MEX whole-body model interface (``mexWholeBodyModel``) for Matlab:

.. code-block:: matlab
   :caption: ``examples/integrateForwardDynamics.m``
   :name: code_example_int_fd

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
   %  Create the initial condition of the state variable "chi" for the integration of
   %  the forward dynamics in state-space form. The state-space form reduces (through
   %  variable substitution) the inhomogeneous second-order ODE to a first-order ODE.
   chi_init = wbm_icub.init_stvChi;

   %% Control torques:
   %  Setup the time-dependent function "fhTrqControl" which describes a forcing
   %  function on the ODEs to control the dynamics of the equation-system. It
   %  refers to the control torques of each time-step t and is needed to calculate
   %  the constraint (contact) forces which influences the outcome of each equation,
   %  the generalized acceleration dv (ddot_q).
   [p_b, R_b] = frame2posRotm(wbm_icub.init_vqT_base);
   g_init = wbm_icub.generalizedBiasForces(R_b, p_b, icub_config.init_state_params.q_j, zeros(icub_model.ndof,1), zeros(6,1));
   len = size(g_init,1);

   % Function handle (fh) for the torque controller function:
   % Note: This function handle has only a very simple dummy-function as controller
   %       (zero torques) that works in this case. For complex scenarios it is
   %       advisable to use a real controller function instead, to avoid integration
   %       errors.
   fhTrqControl = @(t)zeroTrqsController(size(g_init(7:len,1)));

   %% ODE-Solver:
   %  Setup the function handle of the form f(t,chi), where chi refers to the dynamic
   %  state of the system. It evaluates the right side of the nonlinear first-order
   %  ODEs of the form chi' = f(t,chi) and returns a vector of rates of change
   %  (vector of derivatives) which will be integrated by the solver.
   fhFwdDyn = @(t, chi)fastForwardDynamics(t, chi, fhTrqControl, wbm_icub.robot_model, wbm_icub.robot_config);
   %fhFwdDyn = @(t, chi)wbm_icub.forwardDynamics(t, chi, fhTrqControl); % optional

   % specifying the time interval of the integration ...
   sim_time.start = 0.0;
   sim_time.end   = 2.0; %1.5;
   sim_time.step  = 0.01;
   tspan = sim_time.start:sim_time.step:sim_time.end;

   disp('Start the numerical integration...');

   ode_options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4); % setup the error tolerances
   [t, chi]    = ode15s(fhFwdDyn, tspan, chi_init, ode_options); % ODE-Solver
   % or, optional:
   %[t, chi] = fastIntForwardDynamics(fhTrqControl, tspan, chi_init, wbm_icub.robot_model, wbm_icub.robot_config, ode_options);
   %[t, chi] = wbm_icub.intForwardDynamics(tspan, chi_init, fhTrqControl, ode_options);

   save('testTrajectory.mat', 't', 'chi', 'chi_init', 'fhTrqControl', 'icub_model', 'icub_config');
   disp('Numerical integration finished.');

   noi = size(chi,1);
   fprintf('Number of integrations: %d\n', noi);

   %% iCub-Simulator -- Setup the window, the environment and the draw parameters for
   %  the WBM-simulator:

   % create some geometric volume bodies for the simulation environment ...
   rotm_r = eye(3,3); % rectangular orientation
   rotm_2 = [-0.9     0  -0.1;
              0    -0.9     0;
             -0.1     0   0.9];

   vb_objects      = repmat(WBM.vbCuboid, 3, 1);
   vb_objects(1,1) = WBM.vbCuboid(0.1, [0.15; 0.10; 0.61], rotm_r);
   vb_objects(2,1) = WBM.vbCylinder(0.1, 0.2, [-0.2; 0.4; 0.4], rotm_2);
   vb_objects(3,1) = WBM.vbSphere(0.1, [-0.3; 0.3; 0.2], rotm_r);

   show_light = true;
   sim_config = initSimConfigICub(vb_objects, show_light); % shows the simulation with a light scene as default.
   %sim_config = initSimConfigICub(vb_objects, 'DarkScn', show_light); % optional, shows the simulation with a dark scene.
   sim_config = wbm_icub.setupSimulation(sim_config);

   % Define some payload links and link each payload link to a specified volume body:
   % Note: Payload links are usually links of a manipulator (end-effector) or in
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

   sim_config.trajectories = wbm_icub.setTrajectoriesData(lnk_traj, x_out, [1 1 1], [35 35 40]);
   sim_config.show_legend  = true;

   % zoom and shift some specified axes (optional):
   %sim_config.zoomAxes([1 4], [0.9 1.1]);   % axes indices, zoom factors (90%, 110%)
   %sim_config.shiftAxes(4, [0 0.05 -0.12]); %             , shift vectors (x, y, z)

   % show and repeat the simulation 2 times ...
   nRpts = 2;
   wbm_icub.simulateForwardDynamics(x_out, sim_config, sim_time.step, nRpts);

   %% Plot the results -- CoM-trajectory:
   wbm_icub.plotCoMTrajectory(x_out);

   % get the visualization data of the forward dynamics integration for plots
   % and animations:
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
