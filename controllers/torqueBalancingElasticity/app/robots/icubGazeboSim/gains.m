function GAINS = gains(MODEL)
%GAINS defines initial control gains for controlling floating base robots with 
%      stack of task approach.
%
% Format: GAINS = GAINS(MODEL)
%
% Inputs:  - MODEL is a structure defining the robot model.       
%
% Output:  - GAINS it is a structure containing all control gains
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% Config parameters
ndof = MODEL.ndof;

%% Gains for two feet balancing
if sum(MODEL.CONFIG.feet_on_ground) == 2  
    % CoM and angular momentum gains
    gainsPCoM           = diag([40 45 40]);
    gainsDCoM           = 2*sqrt(gainsPCoM);
    gainsPAngMom        = diag([1 5 1]);
    gainsDAngMom        = 2*sqrt(gainsPAngMom);    
    % impedances acting in the null space of the desired contact forces
    impTorso            = [ 20  30  20];
    impArms             = [ 10  10  10   5   5];
    impLeftLeg          = [ 35  40  10  30   5  10];
    impRightLeg         = [ 35  40  10  30   5  10];
end

%% Parameters for one foot balancing
if  sum(MODEL.CONFIG.feet_on_ground) == 1
    % CoM and angular momentum gains
    gainsPCoM          = diag([30 35 30])/5;
    gainsDCoM          = 2*sqrt(gainsPCoM);
    gainsPAngMom       = diag([2.5 5 2.5])/5;
    gainsDAngMom       = 2*sqrt(gainsPAngMom);
    % impedances acting in the null space of the desired contact forces
    impTorso           = [ 10   15   10];
    impArms            = [  5    5    5   2.5   2.5];
    % left and right foot balancing
    if MODEL.CONFIG.feet_on_ground(1) == 1  
        impLeftLeg     = [ 15   15  15  15  5  5];
        impRightLeg    = [ 10   10  15  15  5  5];
    else
        impLeftLeg     = [ 10   10  15  15  5  5];
        impRightLeg    = [ 15   15  15  15  5  5];
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Definition of the impedances and dampings vectors
GAINS.impedances   = [impTorso,impArms,impArms,impLeftLeg,impRightLeg]/5;
GAINS.dampings     = 2*sqrt(GAINS.impedances);

if (size(GAINS.impedances,2) ~= ndof)
    error('Dimension mismatch between ndof and dimension of the variable impedences. Check these variables in the file gains.m');
end

%% Output structure
GAINS.impedances         = diag(GAINS.impedances);
GAINS.dampings           = diag(GAINS.dampings);
GAINS.momentumGains      = [gainsDCoM zeros(3); zeros(3) gainsDAngMom];
GAINS.intMomentumGains   = [gainsPCoM zeros(3); zeros(3) gainsPAngMom];
% gains for feet correction to avoid numerical intrgration errors
GAINS.KpFeet             = 50;
GAINS.reg_gains          = 0.1;

end
