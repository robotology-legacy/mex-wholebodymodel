
%%

%[jl1, jl2] = wholeBodyModel('joint-limits');
load('./jointLimits.mat');

% params.n_dof = 25;                                                              % number of degrees of freedom (actuated)
% params.n_constraint = 2;                                                             % number of constraints

%% initial conditions

% params.torsoInit    = [-10.0 0.0 0.0]';
% params.leftArmInit  = [19.7 29.7 0.0 44.9 0.0]';
% params.rightArmInit = [19.7 29.7 0.0 44.9 0.0]';
% params.leftLegInit  = [40.5 0.1 0.0 -18.5 -5.5 -0.1]';
% params.rightLegInit = [40.5 0.1 0.0 -18.5 -5.5 -0.1]';

params.qjInit = pi*([params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit])/180;

% params.qjDotInit = zeros(params.n_dof,1);
% params.v_baseInit = zeros(6,1);

%% simulation

% params.sim_start_time = 0;
% params.sim_duration = 10;
% params.sim_step = 0.1;
tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);
params.qvInit   = [];
params.current_time = 0;

% params.coef_damp = 0.5;

% visualize_stop = 0;%BOOLEAN
visualize_loop = 0; 
% VISUALIZER_ON = 1;

%% EXPERIMENT
% EXPERIMENT_MODE = 0;

if EXPERIMENT_MODE==0
    ind_exp1max=1;ind_exp2max=1;
else
%     var_exp1 = 'sim_duration';
%     array_var_exp1 = [20  10]';
%     var_exp2 = 'sim_step';
%     array_var_exp2 = [0.1 0.05 0.2]';
%     %
    ind_exp1max = size(array_var_exp1,1);
    ind_exp2max = size(array_var_exp2,1);    
end

%% desired trajectories

% % params.controller.Desired_x_dx_ddx_CoM = @(t)[0.459 0 0 ; 0.068 0 0 ; 0 0 0];
% 
% params.controller.Desired_x_dx_ddx_CoM = @(t)[   0.46     0       0;
%          sin((t+pi/2))*0.04+0.07     cos((t+pi/2))*0.04     -sin((t+pi/2))*0.04;
%               sin(t)*0.02             cos(t)*0.02             -sin(t)*0.02];
 
%% controller

% params.controller.Gains  = [  10    0   2.5  0];% kp ki kd +1
% 
% % Impadances acting in the null space of the desired contact forces 
% params.kImpTorso = [12 8 12];
% % params.kImpTorso = [50 50 50];
% params.kImpArms  = [10 10 10 10 5];
% params.kImpLegs  = [35 50 0.1 30 2 10]; 
% % params.kImpLegs  = [1 1 1 0.1 1 1];

params.controller.Impedances  = [params.kImpTorso,params.kImpArms,params.kImpArms,params.kImpLegs,params.kImpLegs];
params.Impedances  = [params.kImpTorso,params.kImpArms,params.kImpArms,params.kImpLegs,params.kImpLegs];
  
% params.controller.MassMatrix       = zeros(params.n_dof+6);
% params.controller.invMassMatrix    = zeros(params.n_dof+6);
% params.controller.Coriolis         = zeros(params.n_dof+6,1);
% params.controller.Gravitation      = zeros(params.n_dof+6,1);
% params.controller.CentroidalMom    = zeros(6,1);
% params.controller.fkin.rightfoot   = zeros(7,1);
% params.controller.jacobian.feet    = zeros(params.n_constraint*6,params.n_dof+6);                           
% params.controller.JMinvconst       = zeros(params.n_constraint*6,params.n_dof+6);
% params.controller.JMinvJt          = params.controller.JMinvconst * transpose(params.controller.jacobian.feet);                            
% params.controller.jdqd.feet        = zeros(params.n_constraint*6,1);
% params.controller.fkin.com         = zeros(7,1);
% params.controller.jacobian.com     = zeros(params.n_constraint*6,params.n_dof+6);
params.controller.qjInit           = params.qjInit;    






%% results

results_matfile = matfile('results.mat','Writable',true);

if EXPERIMENT_MODE==1
    exp_count=0;
    for ind_exp1 = 1:ind_exp1max
        for ind_exp2 = 1:ind_exp2max
            exp_count=exp_count+1;
            eval(strcat('results.exp',num2str(exp_count),'.',var_exp1,'= array_var_exp1(',num2str(ind_exp1),');'));
            eval(strcat('results.exp',num2str(exp_count),'.',var_exp2,'= array_var_exp2(',num2str(ind_exp2),');'));
        end
    end
end

contactForces = [];
jointTorques = [];
comTraj = [];
% plots
params.PLOT.contactForces = 0;
params.PLOT.jointTorques  = 0;
params.PLOT.comTraj       = 0;
params.PLOT.all           = 0;




%%
wholeBodyModel('update-state',params.qjInit,zeros(params.n_dof,1),zeros(6,1));

[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

params.qvInit = [T_baseInit;params.qjInit;params.qjDotInit;params.v_baseInit];
params.qvInit = [params.qvInit;0;0;0]; %intComError added 
 
forwardDynamics_fun = @(t,qv)cont_forwardDynamics_fun(t,qv,params);