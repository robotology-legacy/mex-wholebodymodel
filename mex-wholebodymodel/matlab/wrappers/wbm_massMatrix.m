function [ M ] = wbm_massMatrix( varargin )
%WMB_MASSMATRIX computes the mass matrix of the floating base system at a
%given joint configuration
%
%   Arguments : 
%       Optimised Mode :  No Arguments
%       Normal Mode : R - rotation from rootLink to world frame (3 x 3)
%                     p - translation from rootLink to world frame (3 x 1)
%                     qj - joint position (NumDoF x 1)
%
%   Returns :   M -  ((NumDoF + 6) x (NumDoF + 6))
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0 
            M = wholeBodyModel('mass-matrix');
        case 3
            M = wholeBodyModel('mass-matrix',reshape(varargin{1},[],1), varargin{2}, varargin{3});
        otherwise
            disp('massMatrix : Incorrect number of arguments, check docs');
    end
end
