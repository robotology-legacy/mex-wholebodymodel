function [ Cqv ] = wbm_generalisedBiasForces( varargin )
%WBM_GENERALISEDBIASFORCES computes the generalised bias forces in the
%   dynamics - which are functions of the joint angles qj, the joint velocities qDot
%   and the floating base velocity vxb
%   Arguments :
%       Optimised Mode : No arguments
%       Normal Mode : R - rotation from rootLink to world frame (3 x 3)
%                     p - translation from rootLink to world frame (3 x 1)
%                     qj - joint angles (NumDoF x 1)
%                     dqj - joint velocities (NumDoF x 1)
%                     vxb - floating base velocity (6 x 1)
%   Returns :   Cqv - Generalised bias forces (6+NumDoF x 1) 
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0 
            Cqv = wholeBodyModel('generalised-forces');
        case 5
            Cqv = wholeBodyModel('generalised-forces',reshape(varargin{1},[],1), varargin{2},varargin{3},varargin{4},varargin{5});
        otherwise
            disp('generalisedBiasForces : Incorrect number of arguments, check docs');
    end       
end

