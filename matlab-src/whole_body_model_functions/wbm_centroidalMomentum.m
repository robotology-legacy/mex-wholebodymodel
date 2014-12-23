function [ H ] = wbm_centroidalMomentum( varargin )
%WMB_CENTROIDALMOMENTUM computes the centroidal momentum of the system -
%functions of the state q, state derivative qDot, and floating base
%velocity vxb
%   Arguments : 
%       Optimised Mode : No arguments
%       Normal Mode :  qj - joint angles (NumDoF x 1)
%                     dqj - joint velocities (NumDoF x 1)
%                     vxb - floating base velocity (6 x 1)
%   Returns :   H - centroidal momentum (6 x 1)
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0 
            H = wholeBodyModel('centroidal-momentum');
        case 3
            H = wholeBodyModel('centroidal-momentum',varargin{1},varargin{2},varargin{3});
        otherwise
            disp('Incorrect number of arguments, check docs');
    end
end

