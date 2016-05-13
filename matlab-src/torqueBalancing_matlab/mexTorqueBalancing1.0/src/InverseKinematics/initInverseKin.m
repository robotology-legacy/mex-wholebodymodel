%% initInverseKin
% this is the inverse kinematic solver used to compute the joints
% references. It uses a double integrator to generate the joints positions
% and velocities from the joints accelerations. The accelerations are
% generated from a task-based structure: first, they must respect the
% contacts constraints. In the null space of this task, the accelerations
% must be compatible with the CoM desired trajectory. Then, as third task,
% they must satisfy a desired posture.
% 
% Outputs:
%
% ikinParam             this is a structure containing the desired joints reference
%                       trajetory and the integration time
%         
% ChiInit [7+2*ndof,1]  this is the initial state of the robot for the
%                       forward dynamics integration
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [ikinParam, ChiInit] = initInverseKin(params)
%% Initial condition for state integration
toll                   = params.pinv_tol;
% generate the initial desired CoM trajectory
desired_x_dx_ddx_CoM   = generTraj(params.CoMInit(1:3), 0, params.trajectory); 

%% Velocity initial conditions for inverse kinematics integration
% define the first task for joints velocities: respect the contact constraints
pinvJcInit          = pinv(params.JcInit,toll);
feetPose            = zeros(6*params.numConstraints,1);
Nullfeet            = eye(params.ndof+6) - pinvJcInit*params.JcInit;

% define the second task for joints velocities: follow a desired dynamics
% for the centroidal momentum
JCentrMom           = [params.JCoMInit(1:3,:); params.JhInit(4:6,:)];
pinvJCentrMom       = pinv(JCentrMom*Nullfeet,toll);
CentroidalDynamics  = [desired_x_dx_ddx_CoM(:,2); zeros(3,1)] - JCentrMom*pinvJcInit*feetPose;
NullCentrMom        = eye(params.ndof+6) - pinvJCentrMom*JCentrMom*Nullfeet;

% define the third task for joints velocities: follow a desired joints
% trajectory
Jjoints             = [zeros(params.ndof,6) eye(params.ndof)];
JointVel            = zeros(params.ndof,1)-Jjoints*(pinvJcInit*feetPose-Nullfeet*pinvJCentrMom*CentroidalDynamics);
pinvJjoints         = pinv(Jjoints*Nullfeet*NullCentrMom,toll);
  
NuThirdTask         = pinvJjoints*JointVel;
NuSecondTask        = pinvJCentrMom*CentroidalDynamics + NullCentrMom*NuThirdTask;
NuFirstTask         = pinvJcInit*feetPose + Nullfeet*NuSecondTask;

% initial condition for state integration 
ChiInit             = [params.BasePoseInit; params.qjInit; NuFirstTask];
 
%% Inverse kinematics integrator
plot_set
% waitbar 
params.wait          = waitbar(0,'Inverse kinematics integration in progress...');
 
ikinParam            = integrateInverseKin(params,ChiInit);

delete(params.wait)       
 
%% Visualize the results of the inverse kinematics
if params.visualize_ikin_results == 1

visInverseKin(params,ikinParam);
end

