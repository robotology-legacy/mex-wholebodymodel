function [ H ] = wbm_centroidalMomentum( varargin )
%WMB_CENTROIDALMOMENTUM computes the centroidal momentum of the system -
%functions of the state q, state derivative qDot, and floating base
%velocity vxb
%   Arguments : 
%       Optimised Mode : No arguments
%       Normal Mode : R - rotation from rootLink to world frame (3 x 3)
%                     p - translation from rootLink to world frame (3 x 1)
%                     qj - joint angles (NumDoF x 1)
%                     dqj - joint velocities (NumDoF x 1)
%                     vxb - floating base velocity (6 x 1)
%   Returns :   H - centroidal momentum (6 x 1)
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0 
            H = wholeBodyModel('centroidal-momentum');
        case 5
            H = wholeBodyModel('centroidal-momentum',reshape(varargin{1},[],1), varargin{2},varargin{3},varargin{4},varargin{5});
        otherwise
            disp('centroidalMomentum : Incorrect number of arguments, check docs');
    end
end

