function [t_kin, joints_traj] = ikin_integrator(params)
% ikin_integrator calculates the desired joints position by
% integrating the desired joints velocity coming from inverse kinematics.
% it uses forward euler integration algorithm.

%% Define the integration time (fixed step)
t_kin = params.tStart:params.euler_step:params.tEnd;
t_kin = t_kin.';
step  = params.euler_step;

%% Initial condition for joints position, velocity and acceleration
y0           = params.ikin_init;

[~,iparam]   = ikin_function(t_kin(1), y0, params );
dy0          = iparam.dq_inv;
ddy0         = iparam.ddq_inv;

% initial condition for errors visualization
delta0       = iparam.delta;
traj0        = iparam.traj;

%% Setup integration
f         =  @(t, ikin) ikin_function( t, ikin, params );

% setup matrix dimensions
dim_t     = length(t_kin);
dim_q     = length(y0);

q_t          = zeros(dim_q,dim_t);
dqj          = zeros(dim_q-7,dim_t);
ddqj         = dqj;

q_t(:,1)     = y0;
dqj(:,1)     = dy0;
ddqj(:,1)    = ddy0;

if params.numConstraints == 2

delta        = zeros(15,dim_t);

elseif params.numConstraints == 1

delta        = zeros(9,dim_t);

end
   
traj         = zeros(18,dim_t);

delta(:,1)   = delta0;
traj(:,1)    = traj0;

%% Euler forward integrator
for k = 2:dim_t
    
 % total state (floating base + joints position)   
 q_t(:,k)        = q_t(:,k-1) + step.*f(t_kin(k-1),q_t(:,k-1));
 
 % desired joints velocity and acceleration
 [~,iparam]      = ikin_function( t_kin(k), q_t(:,k), params); 
 
 dqj(:,k)        = iparam.dq_inv;
 ddqj(:,k)       = iparam.ddq_inv;

 % errors visualization
 delta(:,k)      = iparam.delta;
 traj(:,k)       = iparam.traj;
 
end

%% Joints trajectory definition
% joints position
qj              = q_t(8:end,:);

joints_traj.qj         = qj;
joints_traj.dqj        = dqj;
joints_traj.ddqj       = ddqj;

%% Parameters for visualization
joints_traj.delta      = delta;
joints_traj.traj       = traj;

end
    




