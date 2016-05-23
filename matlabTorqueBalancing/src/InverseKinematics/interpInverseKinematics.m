function  JointReferences = interpInverseKinematics(t,ikinParam)
%INTERPINVERSEKINEMATICS interpolates the joint reference trajectories obtained
%                        from inverse kinematics (using a fixed step integrator)
%                        to make the reference compatible with the variable step
%                        integrator used for the forward dynamics integrations.
%   JointReferences = INTERPINVERSEKINEMATICS(t,ikinParam) performs linear
%   interpolation. The inputs are the forward dynamics integration time t
%   and the structure ikinParam which contains all the parameters from
%   inverse kinematics integration. The output are the Joint References.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
time       = ikinParam.t;

% find in the vector "time" the step which is before the current time t
logic_t    = time<t;
% find the index number
index      = sum(logic_t);
indexNext  = index+1;

if time(indexNext) == t
    
    % the current time step t is also inside the vector time 
  JointReferences.qjRef   = ikinParam.qj(:,indexNext);
  JointReferences.dqjRef  = ikinParam.dqj(:,indexNext);
  JointReferences.ddqjRef = ikinParam.ddqj(:,indexNext);
else
  
  % t is between time(index) and time(indexNext). 
  % it will be interpolated using a line
  DeltaX        = time(indexNext)-time(index);        
  DeltaYPos     = ikinParam.qj(:,indexNext)   - ikinParam.qj(:,index);
  DeltaYVel     = ikinParam.dqj(:,indexNext)  - ikinParam.dqj(:,index);
  DeltaYAcc     = ikinParam.ddqj(:,indexNext) - ikinParam.ddqj(:,index);
  
  % get the tangent of the line
  mPos          = DeltaYPos./DeltaX;
  mVel          = DeltaYVel./DeltaX;
  mAcc          = DeltaYAcc./DeltaX;
  
  % interpolation
  DeltaX0       = t - time(index);
  
  JointReferences.qjRef   = ikinParam.qj(:,index)     + mPos*DeltaX0;
  JointReferences.dqjRef  = ikinParam.dqj(:,index)    + mVel*DeltaX0;
  JointReferences.ddqjRef = ikinParam.ddqj(:,index)   + mAcc*DeltaX0;
end  
end
