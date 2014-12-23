function [ Cqv ] = wbm_generalisedBiasForces( varargin )
%WBM_GENERALISEDBIASFORCES computes the generalised bias forces in the
%   dynamics - which are functions of the joint angles qj, the joint velocities qDot
%   and the floating base velocity vxb
%   Arguments :
%       Optimised Mode : No arguments
%       Normal Mode : qj - joint angles (NumDoF x 1)
%                     dqj - joint velocities (NumDoF x 1)
%                     vxb - floating base velocity (6 x 1)
%   Returns :   Cqv - Generalised bias forces (6+NumDoF x 1) 
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0 
            Cqv = wholeBodyModel('generalised-forces');
        case 3
            Cqv = wholeBodyModel('generalised-forces',varargin{1},varargin{2},varargin{3});
        otherwise
            disp('Incorrect number of arguments, check docs');
    end       
end

