function  JointReferences = interpInverseKinematics(t,ikin)
%INTERPINVERSEKINEMATICS interpolates the joint reference trajectory obtained
%                        from inverse kinematics (which uses a fixed step integrator)
%                        to make the reference compatible with the variable step
%                        integrator used in the forward dynamics integrations.
%
%   JointReferences = INTERPINVERSEKINEMATICS(t,ikin) performs a linear
%   interpolation. The inputs are the forward dynamics integration time t
%   and the structure ikin which contains all the parameters from
%   inverse kinematics integration. The output are the Joint References.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
time       = ikin.t;

% find in the vector "time" the step which is before the current time t
logic_t    = time<t;
% find the index number
index      = sum(logic_t);
indexNext  = index+1;

if time(indexNext) == t
    
    % the current time step t is also inside the vector time
    JointReferences.qjRef   = ikin.qj(:,indexNext);
    JointReferences.dqjRef  = ikin.dqj(:,indexNext);
    JointReferences.ddqjRef = ikin.ddqj(:,indexNext);
else
    
    % t is between time(index) and time(indexNext).
    % it will be interpolated using a line
    DeltaX        = time(indexNext)-time(index);
    DeltaYPos     = ikin.qj(:,indexNext)   - ikin.qj(:,index);
    DeltaYVel     = ikin.dqj(:,indexNext)  - ikin.dqj(:,index);
    DeltaYAcc     = ikin.ddqj(:,indexNext) - ikin.ddqj(:,index);
    
    % get the tangent of the line
    mPos          = DeltaYPos./DeltaX;
    mVel          = DeltaYVel./DeltaX;
    mAcc          = DeltaYAcc./DeltaX;
    
    % interpolation
    DeltaX0       = t - time(index);
    
    JointReferences.qjRef   = ikin.qj(:,index)     + mPos*DeltaX0;
    JointReferences.dqjRef  = ikin.dqj(:,index)    + mVel*DeltaX0;
    JointReferences.ddqjRef = ikin.ddqj(:,index)   + mAcc*DeltaX0;
end
end
