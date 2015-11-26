% namespaces:
import WBM.*
import WBM.utilities.*


%% Initialization of the WBM:
%
iCub_model  = wbmBaseModelParams; 
iCub_config = wbmHumanoidConfig;
% base robot config:
iCub_config.ndof          = 25; 
iCub_config.nCstrs        = 2; 
iCub_config.cstrLinkNames = {'l_sole', 'r_sole'};
iCub_config.dampCoeff     = 0.0; %0.5;
% base model parameters:
%iCub_model.urdfRobot    = 'icubGazeboSim';
%iCub_model.urdfLinkName = 'l_sole';
%iCub_model.wf_R_rootLnk = zeros(3,3);
%iCub_model.wf_p_rootLnk = zeros(3,1);
iCub_model.g_wf         = [0; 0; 9.81]; %zeros(6+iCub_config.ndof,1);
% body positions of the iCub-Robot (in degrees):
% (this configuration assumes an iCub-Robot with 25 DoFs.)
iCub_config.pos_torso    = [-10.0; 0.0; 0.0];
iCub_config.pos_leftArm  = [ -19.7; 29.7; 0.0; 44.9; 0.0];
iCub_config.pos_leftLeg  = [ 25.5; 0.1; 0.0; -38.5; -5.5; -0.1];
iCub_config.pos_rightArm = iCub_config.pos_leftArm;
iCub_config.pos_rightLeg = iCub_config.pos_leftLeg;
% init-state parameters:
iCub_config.initStateParams.q_j  = [iCub_config.pos_torso; iCub_config.pos_leftArm; iCub_config.pos_rightArm; ...
                                    iCub_config.pos_leftLeg; iCub_config.pos_rightLeg] * (pi/180.0); % in radians
iCub_config.initStateParams.dq_j = zeros(iCub_config.ndof,1);
%iCub_config.initStateParams.dx_b    = zeros(3,1);
%iCub_config.initStateParams.omega_b = zeros(3,1);

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
chi_init = wbm_iCub.getStateVector();


% The function tau(t) is usually called a "forcing term" on the ode. 


%% Control torques:
%  Setup the time-dependent variable "tau" that refers to the control torques of each
%  equation in the ODEs. This variable is a forcing term that is needed in the
%  forward dynamic function to calculate the constraint forces f_c and the
%  generalized acceleration dv (q_ddot).
vqT_b_init = chi_init(1,1);
[p_b, R_b] = frame2posRotm(vqT_b_init);
g_init = wbm_iCub.generalBiasForces(R_b, p_b, iCub_config.initStateParams.q_j, zeros(25,1), zeros(6,1));

ctrlTrqs.tau = @(t)zeros(size(g_init(7:end)));

%% ODE-Solver:
%  Setup the function handle of the form f(t,chi) where chi refers to the dynamic state
%  of the system. It evaluates the right side of the nonlinear first-order ODEs of the
%  form chi' = f(t,chi) and returns a vector of rates of change (vector of derivatives)
%  that will be integrated by the solver.
fwdDynFunc = @(t, chi)wbm_iCub.forwardDynamics(t, chi, ctrlTrqs);
% specifying the time interval of the integration ...
sim_time.start = 0.0;
sim_time.end   = 2.0; %1.5;
sim_time.step  = 0.01;
tspan = [sim_time.start:sim_time.step:sim_time.end];

disp('Start the numerical integration:');

ode_options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);           % setup the error tolerances ...
[t, chi]    = ode15s(fwdDynFunc, tspan, chi_init, ode_options); % ODE-Solver

save('testTrajectory.mat', 't', 'chi', 'ctrlTrqs', 'iCub_model', 'iCub_config');
disp('Numerical integration finished.');

%% Setup the window and plot parameters for the WBM-simulator:
sim_config = iCubSimConfig;
wbm_iCub.setupSimulation(sim_config);





%% Old code: -------------------------------------------------------------------

%% initialise mexWholeBodyModel
wbm_modelInitialise('icubGazeboSim');


%wbm_setWorldLink('codyco_balancing_world',eye(3),[0 0 0]',[-9.81,0,0]');
%wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[ 0,0,-9.81]');

%if(exist('storedTestTrajectory.mat','file')==0)
    %wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[-9.81, 0 , 0]');

    %% setup params
    params.ndof = 25;
    param.dampingCoeff = 0.5;

    %% initial conditions
    % this is assuming a 25DoF iCub
    params.torsoInit    = [-10.0  0.0   0.0]';%[-10.0 0.0 0.0];
    params.leftArmInit  = [ -19.7  29.7  0.0  44.9  0.0]';%params.leftArmInit = zeros(size(params.leftArmInit));
    params.rightArmInit = [ -19.7  29.7  0.0  44.9  0.0]';%params.rightArmInit = zeros(size(params.rightArmInit));
    %params.leftLegInit  = [ 25.5   0.1  0.0 -18.5 -5.5 -0.1]'
    params.leftLegInit  = [ 25.5   0.1  0.0 -38.5 -5.5 -0.1]';%params.leftLegInit = zeros(size(params.leftLegInit));
    params.rightLegInit = [ 25.5   0.1  0.0 -38.5 -5.5 -0.1]';%params.rightLegInit = zeros(size(params.rightLegInit));

    params.qjInit = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit] * (pi/180);
    %params.qjInit = zeros(size(params.qjInit));
    params.dqjInit = zeros(params.ndof,1);
    %params.x_bInit =  zeros(3,1);
    %params.qt_bInit = zeros(4,1);
    params.dx_bInit = zeros(3,1);
    params.omega_bInit = zeros(3,1);
    params.dampingCoeff = 0.00;%0.75;
    
    wbm_setWorldFrame(eye(3),zeros(3,1),[ 0,0,-9.81]');
    wbm_updateState(params.qjInit,zeros(params.ndof,1),zeros(6,1));
    [rot,pos] = wbm_getWorldFrameFromFixedLink('l_sole',params.qjInit);
    % fprintf('Converting to a set world frame... \n');
    wbm_setWorldFrame(rot,pos,[ 0,0,-9.81]');
    
    [qj,T_bInit,dqj,vb] = wbm_getState();

    qt_b_mod_s = T_bInit(4);
    qt_b_mod_r = T_bInit(5:end);
    R_b = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2 * skew(qt_b_mod_r)^2;
    p_b = T_bInit(1:3);
    
    params.chiInit = [T_bInit;params.qjInit;...
                        params.dx_bInit;params.omega_bInit;params.dqjInit];

    %% contact constraints                
    %params.constraintLinkNames = {'l_sole','r_sole'};                
    %params.constraintLinkNames = {};
    
    params.constraintLinkNames = {'l_sole','r_sole'};                    
    params.numConstraints= length(params.constraintLinkNames);
    %% control torques
    gInit = wbm_generalisedBiasForces(R_b,p_b,params.qjInit,zeros(25,1),zeros(6,1));
  %  params.tau = @(t)gInit(7:end);
     params.tau = @(t)zeros(size(gInit(7:end)));
    %params.tau = @(t)1.5*ones(size(gInit(7:end)));
    %params.tau = @(t)0.01*ones(size(gInit(1:params.ndof)));%gInit(1:params.ndof);

    %% setup integration
    forwardDynFunc = @(t,chi)forwardDynamics(t,chi,params);
    params.tStart = 0;
    params.tEnd = 2;%1.5;
    params.sim_step = 0.01;


    %% integrate forward dynamics
    disp('starting numerical integration');
    options = odeset('RelTol',1e-2,'AbsTol',1e-4);
    %[t,chi] = ode15s(forwardDynFunc,[params.tStart params.tEnd],params.chiInit,options);
    [t,chi] = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);

    save('storedTestTrajectory.mat','t','chi','params');
    disp('numerical integration complete..rendering.');
% else
%     load('storedTestTrajectory.mat');
% end
temp = 1;

while(temp<10)
    figure; clf;
    % initialize GUI
    figure_main = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
        'Position', [50,400,600,650]);
    params.figure_main = figure_main;
    set(figure_main, 'MenuBar', 'none', 'BackingStore', 'off');
    set(figure_main, 'BackingStore', 'off');
    % plot_main = subplot('Position', [0.1,0.25,0.80,0.70]);
    params.plot_main =zeros(1,4);
    plot_pos = [0.51,0.20,0.45,0.40;
                0.01,0.20,0.45,0.40;
                0.51,0.62,0.45,0.40;
                0.01,0.62,0.45,0.40];

    for ii=1:4
        params.plot_main(ii) = subplot('Position', plot_pos(ii,:));
        params.plot_objs{ii} = plot3(0,0,0,'.');
        axis([-0.5 0.5 -0.42 0.58 0 1]); hold on;
        patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);
        set(gca,'Color',[0.8 0.8 0.8]);
        set(gca,'XColor',[0.8 0.8 0.8]);
        set(gca,'YColor',[0.8 0.8 0.8]);
        set(gca,'ZColor',[0.8 0.8 0.8]);
        set(gca,'xdir','reverse')
        set(gca, 'drawmode', 'fast');
        params.draw_init = 1;
        rotate3d(gca,'on');

        figure(figure_main);
    end
    axes(params.plot_main(1))



    %% plot results
    % CoM trajectory
    ndof = params.ndof;
    x_b = chi(:,1:3,:);
    qt_b = chi(:,4:7);
    qj = chi(:,8:ndof+7);

    visualizeForwardDynamics([x_b,qt_b,qj],t,params);
    temp = 10;%temp+1;
    pause;
end
    figure(2);
    plot3(x_b(:,1),x_b(:,2),x_b(:,3));hold on;
    plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro');
    grid on;
    axis square;
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
 