function gainsInit = gains(CONFIG)
%GAINS generates the initial gains matrices for both the
%      momentum task and the postural task. 
%      GAINS also apply a modification of the postural task to
%      assure the system's asymptotic stability.
%
%      gains = GAINS(config) takes as an input the structure CONFIG, which
%      contains all the utility parameters, and the structure DYNAMICS 
%      which contains the robot dynamics. The output is the structure 
%      GAINSINIT, which contains the initial gains matrices.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters
ndof           = CONFIG.ndof;
postCorrection = CONFIG.postCorrection;
DYNAMICS       = CONFIG.initDynamics;
pinv_damp      = CONFIG.pinv_damp;
%pinv_tol      = CONFIG.pinv_tol;

% Dynamics parameters
M              = DYNAMICS.M;
Jc             = DYNAMICS.Jc;

% General parameters
S              = [zeros(6,ndof);
                  eye(ndof)];

%% Gains for two feet on the ground
if sum(CONFIG.feet_on_ground) == 2
    
    gainsPCoM           = diag([45 50 40]);
    gainsDCoM           = 2*sqrt(gainsPCoM);
    gainsPAngMom        = diag([5 10 5]);
    gainsDAngMom        = 2*sqrt(gainsPAngMom);

% impedances acting in the null space of the desired contact forces 
    impTorso            = [ 40  40  40]; 
    impArms             = [ 10  10   10   5   5];
    impLeftLeg          = [ 35  40   10  30   5  10]; 
    impRightLeg         = [ 35  40   10  30   5  10];       
end

%% Parameters for one foot on the ground
if  sum(CONFIG.feet_on_ground) == 1
 
     gainsPCoM          = diag([40 45 40]);
     gainsDCoM          = 2*sqrt(gainsPCoM);
     gainsPAngMom       = diag([5 10 5]);
     gainsDAngMom       = 2*sqrt(gainsPAngMom);
   
% impedances acting in the null space of the desired contact forces 
     impTorso           = [ 20   20   20]; 
     impArms            = [ 15  15   45   5   5];

if CONFIG.feet_on_ground(1) == 1
    
     impLeftLeg         = [ 70   70  65  30  10  10];  
     impRightLeg        = [ 20   20  20  10  10  10];   
else
     impLeftLeg         = [ 20   20  20  10  10  10];
     impRightLeg        = [ 70   70  65  30  10  10]; 
end
end

%% Definition of the impedances and dampings vectors 
gainsInit.impedances    = [impTorso,impArms,impArms,impLeftLeg,impRightLeg];
gainsInit.dampings      = 2*sqrt(gainsInit.impedances);

if (size(gainsInit.impedances,2) ~= ndof)
    
  error('Dimension mismatch between ndof and dimension of the variable impedences. Check these variables in the file gains.m');    
end

%% MOMENTUM AND POSTURAL GAINS
gainsInit.impedances         = diag(gainsInit.impedances);
gainsInit.dampings           = diag(gainsInit.dampings); 
gainsInit.MomentumGains      = [gainsDCoM zeros(3); zeros(3) gainsDAngMom];
gainsInit.intMomentumGains   = [gainsPCoM zeros(3); zeros(3) gainsPAngMom];

% Desired shape for the state matrix of the linearized system, for gains tuning procedure
gainsInit.KSdes              = gainsInit.impedances;
gainsInit.KDdes              = gainsInit.dampings;

%% Correction of postural task that ensures the asymptotic stability of the joint space
if postCorrection == 1 

JcMinv                  = Jc/M;
JcMinvS                 = JcMinv*S;
pinvJcMinvS             = JcMinvS'/(JcMinvS*JcMinvS' + pinv_damp*eye(size(JcMinvS,1)));
%pinvJcMinvS            = pinv(JcMinvS,pinv_tol);
Mbar                    = M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end);

% damped null space
NullLambda              = eye(ndof)-pinvJcMinvS*JcMinvS;

% this is the term added to the postural task to assure the dynamics to be
% asymptotically stable
gainsInit.posturalCorr  = NullLambda*Mbar;

else  
gainsInit.posturalCorr  = eye(ndof);
end

end
