function [jlim_lower, jlim_upper]  = wbm_jointLimits()
    % wbm_jointLimits returns the lower and upper limits of the joint positions of the robot model.
    %
    %   INPUT ARGUMENTS:  none
    %
    %   OUTPUT ARGUMENTS:
    %       jlim_lower -- (nDoF x 1) lower joint limits vector
    %       jlim_upper -- (nDoF x 1) upper joint limits vector
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    [jlim_lower, jlim_upper] = mexWholeBodyModel('joint-limits');
end
