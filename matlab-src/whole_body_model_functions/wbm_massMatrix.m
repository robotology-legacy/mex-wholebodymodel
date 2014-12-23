function [ M ] = wbm_massMatrix( varargin )
%WMB_MASSMATRIX computes the mass matrix of the floating base system at a
%given joint configuration
%
%   Arguments : 
%       Optimised Mode :  No Arguments
%       Normal Mode :   qj - joint position (NumDoF x 1)
%
%   Returns :   M -  ((NumDoF + 6) x (NumDoF + 6))
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0 
            M = wholeBodyModel('mass-matrix');
        case 1
            M = wholeBodyModel('mass-matrix',varargin{1});
        otherwise
            disp('Incorrect number of arguments, check docs');
    end
end
