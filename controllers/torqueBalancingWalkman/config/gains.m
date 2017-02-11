function gainsInit = gains(CONFIG)
%GAINS generates initial control gains for both the momentum task 
%(primary task in SoT controller) and the postural task.
%
% gains = GAINS(CONFIG) takes as an input the structure CONFIG, which
% contains all the robot configuration parameters. The output is the structure 
% gainsInit, which contains the initial gains matrices.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters
ndof = CONFIG.ndof;

%% Gains for two feet on ground
if sum(CONFIG.feet_on_ground) == 2
    
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

%% Parameters for one foot on ground
if  sum(CONFIG.feet_on_ground) == 1
    
    % CoM and angular momentum gains
    gainsPCoM          = diag([30 35 30]);
    gainsDCoM          = 2*sqrt(gainsPCoM);
    gainsPAngMom       = diag([2.5 5 2.5]);
    gainsDAngMom       = 2*sqrt(gainsPAngMom);
    
    % impedances acting in the null space of the desired contact forces
    impTorso           = [ 10   15   10];
    impArms            = [  5    5    5   2.5   2.5];
    
    if CONFIG.feet_on_ground(1) == 1
        
        impLeftLeg     = [ 15   15  15  15  5  5];
        impRightLeg    = [ 10   10  15  15  5  5];
    else
        impLeftLeg     = [ 10   10  15  15  5  5];
        impRightLeg    = [ 15   15  15  15  5  5];
    end
end

%% Definition of the impedances and dampings vectors
gainsInit.impedances   = [impTorso,impArms,impArms,impLeftLeg,impRightLeg];
gainsInit.dampings     = 2*sqrt(gainsInit.impedances);

if (size(gainsInit.impedances,2) ~= ndof)
    
    error('Dimension mismatch between ndof and dimension of the variable impedences. Check these variables in the file gains.m');
end

%% Momentum and postural gains
gainsInit.impedances         = diag(gainsInit.impedances);
gainsInit.dampings           = diag(gainsInit.dampings);
gainsInit.momentumGains      = [gainsDCoM zeros(3); zeros(3) gainsDAngMom];
gainsInit.intMomentumGains   = [gainsPCoM zeros(3); zeros(3) gainsPAngMom];

% gains for feet correction to avoid numerical intrgration errors
gainsInit.corrPosFeet        = 100;

end
