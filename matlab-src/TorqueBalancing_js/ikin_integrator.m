function [t_kin, joints_traj] = ikin_integrator(params)
% ikin_integrator 
% calculates the desired joints position and velocity by
% integrating the desired joints acceleration coming from inverse kinematics.
% it uses forward euler integration algorithm.

%% Define the integration time (fixed step)
t_kin = params.tStart:params.euler_step:params.tEnd;
t_kin = t_kin';
step  = params.euler_step;
ndof  = params.ndof;

%% Initial condition for joints position, velocity and acceleration
y0           = params.ikin_init;

[~,iparam]   = ikin_function(t_kin(1), y0, params );
ddy0         = iparam.ddq_inv;

% initial condition for errors visualization
delta0       = iparam.delta;
traj0        = iparam.traj;

%% Setup integration
f         =  @(t, ikin) ikin_function( t, ikin, params );

% setup matrix dimensions
dim_t     = length(t_kin);
dim_q     = length(y0);

qj        = zeros(ndof,dim_t);
dqj       = zeros(ndof,dim_t);
ddqj      = dqj;
q_total   = zeros(dim_q,dim_t);

q_total(:,1)     = y0;
ddqj(:,1)        = ddy0;

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
    
 % total state (floating base + joints position + vi)   
 q_total(:,k)    = q_total(:,k-1) + step.*f(t_kin(k-1),q_total(:,k-1));
 
 % desired joints velocity and acceleration
 [~,iparam]      = ikin_function(t_kin(k), q_total(:,k), params); 
 
 qj(:,k)         = q_total(8:7+ndof,k);
 dqj(:,k)        = q_total(14+ndof:end,k);
 ddqj(:,k)       = iparam.ddq_inv;

 % errors visualization
 delta(:,k)      = iparam.delta;
 traj(:,k)       = iparam.traj;
 
end

%% Joints trajectory definition
% joints position
joints_traj.qj         = qj;
joints_traj.dqj        = dqj;
joints_traj.ddqj       = ddqj;

%% Parameters for visualization
joints_traj.delta      = delta;
joints_traj.traj       = traj;

end
    
