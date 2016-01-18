function [t_kin,joints_traj] = ikin_integrator(params)
%% ikin_integrator 
%  Calculates the desired joints position and velocity by integrating the desired
%  joints acceleration coming from inverse kinematics. It uses forward euler integration algorithm.
%  The output are:
%
%  t_kin         which is the vector of integration time
%
%  joints_traj   which is a structure containing joint referece position,
%                velocity and acceleration and CoM desired trajectory

%% Define the integration time (fixed step)
t_kin = params.tStart:params.euler_step:params.tEnd;
t_kin = t_kin';
step  = params.euler_step;
ndof  = params.ndof;

%% Initial condition for joints position, velocity and acceleration
ikin_init        = params.ikin_init;

[~,ikinParam]    = ikin_function(t_kin(1), ikin_init, params );
ddqjDesInit      = ikinParam.ddqjDes;

% initial condition for errors visualization
ikinDeltaInit    = ikinParam.Delta;
trajInit         = ikinParam.traj;

%% Setup integration
integratedFunction  =  @(t,ikin) ikin_function( t,ikin,params );

% setup matrix dimensions
dim_t     = length(t_kin);
dim_qj    = length(ikin_init);

qj        = zeros(ndof,dim_t);
dqj       = qj;
ddqj      = dqj;

qState    = zeros(dim_qj,dim_t);

qState(:,1)   = ikin_init;
qj(:,1)       = ikin_init(8:7+ndof);
dqj(:,1)      = ikin_init(14+ndof:end);
ddqj(:,1)     = ddqjDesInit;

if params.numConstraints == 2

ikinDelta        = zeros(15,dim_t);

elseif params.numConstraints == 1

ikinDelta        = zeros(9,dim_t);

end
   
traj             = zeros(18,dim_t);

ikinDelta(:,1)   = ikinDeltaInit;
traj(:,1)        = trajInit;

%% Euler forward integrator
for k = 2:dim_t
    
% total state (floating base pose + joints position + Nu)   
 qState(:,k)   = qState(:,k-1) + step.*integratedFunction(t_kin(k-1),qState(:,k-1));
 
% desired joints position, velocity and acceleration
 [~,ikinParam] = ikin_function(t_kin(k), qState(:,k), params); 
 
 qj(:,k)       = qState(8:7+ndof,k);
 dqj(:,k)      = qState(14+ndof:end,k);
 ddqj(:,k)     = ikinParam.ddqjDes;

% errors visualization
 ikinDelta(:,k)  = ikinParam.Delta;
 traj(:,k)       = ikinParam.traj;
 
end

%% Joints trajectory definition
joints_traj.qj         = qj;
joints_traj.dqj        = dqj;
joints_traj.ddqj       = ddqj;

%% Parameters for visualization
joints_traj.delta      = ikinDelta;
joints_traj.traj       = traj;

end
    
