function  jointReferences = interpInverseKinematics(t,IKIN)
%INTERPINVERSEKINEMATICS interpolates the joint reference trajectory obtained
%                        from inverse kinematics (which uses a fixed step integrator)
%                        to make the reference compatible with the variable step
%                        integrator used in the forward dynamics integrations.
%
% jointReferences = INTERPINVERSEKINEMATICS(t,IKIN) performs a linear
% interpolation. The inputs are the forward dynamics integration time t
% and the structure IKIN which contains all the parameters from
% inverse kinematics integration. The output are the joint references.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% initial parameters
time       = IKIN.t;

% find in the vector "time" the step which is before the current time t
logic_t    = time<t;
% find the index number
index      = sum(logic_t);
indexNext  = index+1;

if time(indexNext) == t
    
    % the current time step t is also inside the vector time
    jointReferences.qjRef   = IKIN.qj(:,indexNext);
    jointReferences.dqjRef  = IKIN.dqj(:,indexNext);
    jointReferences.ddqjRef = IKIN.ddqj(:,indexNext);
else
    
    % t is between time(index) and time(indexNext).
    % it will be interpolated using a line
    DeltaX        = time(indexNext)-time(index);
    DeltaYPos     = IKIN.qj(:,indexNext)   - IKIN.qj(:,index);
    DeltaYVel     = IKIN.dqj(:,indexNext)  - IKIN.dqj(:,index);
    DeltaYAcc     = IKIN.ddqj(:,indexNext) - IKIN.ddqj(:,index);
    
    % get the tangent of the line
    mPos          = DeltaYPos./DeltaX;
    mVel          = DeltaYVel./DeltaX;
    mAcc          = DeltaYAcc./DeltaX;
    
    % interpolation
    DeltaX0       = t - time(index);
    
    jointReferences.qjRef   = IKIN.qj(:,index)     + mPos*DeltaX0;
    jointReferences.dqjRef  = IKIN.dqj(:,index)    + mVel*DeltaX0;
    jointReferences.ddqjRef = IKIN.ddqj(:,index)   + mAcc*DeltaX0;
end
end
