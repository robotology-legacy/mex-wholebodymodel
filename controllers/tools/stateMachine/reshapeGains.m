function GAINS = reshapeGains(gainVect,MODEL)
%RESHAPEGAINS reshapes gain matrices for CoM and joint position.
%
% Format: GAINS = reshapeGains(gainVect,CONFIG)
%
% Inputs:  - gainVect [3+ndof x 1] is a vector containing CoM and joints gains; 
%          - MODEL is a structure defining the robot model. 
%
% Output:  - GAINS it is a structure containing all control gains
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% initial gains
GAINS                    = gains(MODEL);
% update CoM gains
gainsPCoM                = diag(gainVect(1:3));
gainsDCoM                = 2*sqrt(gainsPCoM);
% update impedance and damping
GAINS.impedances         = diag(gainVect(4:end));
GAINS.dampings           = 2*sqrt(GAINS.impedances);
% update momentum gains
GAINS.momentumGains      = [gainsDCoM zeros(3); zeros(3) GAINS.momentumGains(4:end,4:end)];
GAINS.intMomentumGains   = [gainsPCoM zeros(3); zeros(3) GAINS.intMomentumGains(4:end,4:end)];

end
