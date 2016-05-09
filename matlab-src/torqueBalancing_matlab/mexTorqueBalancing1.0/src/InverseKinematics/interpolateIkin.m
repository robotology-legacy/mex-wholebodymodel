%% interpolateIkin
%
%  Interpolates the trajectory obtained with inverse kinematic integrator
%  to fit the step of state integrator. 
%  The outputs are:
%  jointReferences  which is a structure containing the desired joint
%                   position, velocity and acceleration;
%
function  jointReferences = interpolateIkin(t,ikin,params)

%% Trajectory interpolation 
time = ikin.t;

% find in the vector "time" the step which is before t (current time)
logic_t    = time<t;
index      = sum(logic_t);
index_next = index+1;

if time(index_next) == t
    
    jointReferences.qjRef   = ikin.qj(:,index_next);
    jointReferences.dqjRef  = ikin.dqj(:,index_next);
    jointReferences.ddqjRef = ikin.ddqj(:,index_next);

else
  
  % if t is between time(index) and time(index_next), it will be interpolated
  % using a line
  x_delta    = time(index_next)-time(index);
        
  yPos_delta   = ikin.qj(:,index_next)   - ikin.qj(:,index);
  yVel_delta   = ikin.dqj(:,index_next)  - ikin.dqj(:,index);
  yAcc_delta   = ikin.ddqj(:,index_next) - ikin.ddqj(:,index);
  
  % tangent calculation
  mPos         = yPos_delta./x_delta;
  mVel         = yVel_delta./x_delta;
  mAcc         = yAcc_delta./x_delta;
  
  % desired value interpolation
  x0_delta   = t - time(index);
  
  jointReferences.qjRef   = ikin.qj(:,index)     + mPos*x0_delta;
  jointReferences.dqjRef  = ikin.dqj(:,index)    + mVel*x0_delta;
  jointReferences.ddqjRef = ikin.ddqj(:,index)   + mAcc*x0_delta;

end
  

end