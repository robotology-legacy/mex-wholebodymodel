function  [jointReferences, errorCoM] = interpolation(t,params,xCoM)
%% interpolation 
%  Interpolates the trajectory obtained with inverse kinematic integrator
%  to fit the step of state integrator. 
%  The outputs are:
%  jointReferences  which is a structure containing the desired joint
%                   position, velocity and acceleration;
%
%  errorCoM         [3x1] which is the CoM position error
%
%% Trajectory interpolation 
time = params.t_kin;

% find in the vector "time" the step which is before t (current time)
logic_t    = time<t;
index      = sum(logic_t);
index_next = index+1;

if time(index_next) == t
    
    jointReferences.qjDes   = params.joints_traj.qj(:,index_next);
    jointReferences.dqjDes  = params.joints_traj.dqj(:,index_next);
    jointReferences.ddqjDes = params.joints_traj.ddqj(:,index_next);
    CoM_Des                 = params.joints_traj.traj(1:3,index_next);

else
  
  % if t is between time(index) and time(index_next), it will be interpolated
  % using a line
  x_delta    = time(index_next)-time(index);
        
  yPos_delta   = params.joints_traj.qj(:,index_next)   - params.joints_traj.qj(:,index);
  yVel_delta   = params.joints_traj.dqj(:,index_next)  - params.joints_traj.dqj(:,index);
  yAcc_delta   = params.joints_traj.ddqj(:,index_next) - params.joints_traj.ddqj(:,index);
  
  CoM_delta    = params.joints_traj.traj(1:3,index_next) - params.joints_traj.traj(1:3,index);
  
  % tangent calculation
  mPos         = yPos_delta./x_delta;
  mVel         = yVel_delta./x_delta;
  mAcc         = yAcc_delta./x_delta;
  mCoM         = CoM_delta./x_delta;
  
  % desired value interpolation
  x0_delta   = t - time(index);
  
  jointReferences.qjDes   = params.joints_traj.qj(:,index)     + mPos*x0_delta;
  jointReferences.dqjDes  = params.joints_traj.dqj(:,index)    + mVel*x0_delta;
  jointReferences.ddqjDes = params.joints_traj.ddqj(:,index)   + mAcc*x0_delta;
  CoM_Des                 = params.joints_traj.traj(1:3,index) + mCoM*x0_delta;

end
  
% CoM position error 
errorCoM = abs(CoM_Des-xCoM);

end