function [jl_lower,jl_upper]  = wbm_jointLimits( varargin )
%WBM_JOINTLIMITS Obtains the joint limits for the chosen robot
%
%   Arguments : 
%       Optimised Mode : No arguments
%       Normal Mode :  No arguments
%   Returns :   jl_lower - lower joint limits (NumDoF x 1)
%               jl_upper - upper joint limits (NumDoF x 1)
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0
            [jl_lower,jl_upper]  = wholeBodyModel('joint-limits');
        otherwise
             disp('jointLimits : Incorrect number of arguments, check docs'); 
    end
end

