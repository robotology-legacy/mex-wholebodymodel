
%% GENERAL
params.n_dof = 25;                          % number of degrees of freedom (actuated)
params.n_constraint = 2;                    % number of constraints
params.coef_damp = 0;                     % Damping on the joints

%% INITIAL CONDITIONS
params.torsoInit    = [-10.0 0.0 0.0]';
params.leftArmInit  = [19.7 29.7 0.0 44.9 0.0]';
params.rightArmInit = [19.7 29.7 0.0 44.9 0.0]';
params.leftLegInit  = [40.5 0.1 0.0 -18.5 -5.5 -0.1]';
params.rightLegInit = [40.5 0.1 0.0 -18.5 -5.5 -0.1]';

params.qjDotInit = zeros(params.n_dof,1);
params.v_baseInit = zeros(6,1);

%% SIMULATION

params.sim_start_time = 0;
params.sim_duration = 10;
params.sim_step = 0.1;

params.visualize_stop = 0;%not used          
params.visualize_loop = 0;          % to repeat the visualization process; can be accessed from GUI 

%% EXPERIMENT

VISUALIZER_ON = 1;                  % to turn on/off visualization during experiments
EXPERIMENT_MODE = 0;                % When 1; variables below can be set to conduct experiments

% Possible choices for variables to test are:
% 'sim_duration' 'sim_step' 'coef_damp' 'controller.Gains' 'torsoInit' 
% 'leftArmInit' 'rightArmInit' 'leftLegInit' 'rightLegInit' ''
% ''

var_exp1 = 'sim_duration';
array_var_exp1 = [20  10]';
var_exp2 = 'sim_step';
array_var_exp2 = [0.1 0.05 0.2]';

% params.controller.Desired_x_dx_ddx_CoM = @(t)[0.459 0 0 ; 0.068 0 0 ; 0 0 0];

params.controller.Desired_x_dx_ddx_CoM = @(t)[   0.46     0       0;
         sin((t+pi/2))*0.04+0.07     cos((t+pi/2))*0.04     -sin((t+pi/2))*0.04;
              sin(t)*0.02             cos(t)*0.02             -sin(t)*0.02];

params.controller.Gains  = [  10    0   2.5  0];% kp ki kd +1

% Impadances acting in the null space of the desired contact forces 
params.kImpTorso = [12 8 12];
% params.kImpTorso = [50 50 50];
params.kImpArms  = [10 10 10 10 5];
params.kImpLegs  = [35 50 0.1 30 2 10]; 
% params.kImpLegs  = [1 1 1 0.1 1 1];
%%

disp('Parameters set.');

update_parameters;

