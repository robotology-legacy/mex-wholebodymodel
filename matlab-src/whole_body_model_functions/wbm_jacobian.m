function [ J ] = wbm_jacobian( varargin )
%WMB_JACOBIAN computes the Jacobian to a desired link (frame) at a given
%joint configuration
%velocity vxb
%   Arguments : 
%       Optimised Mode :  link_name - string matching URDF name of the link (frame)
%       Normal Mode :   link_name - string matching URDF name of the link (frame)
%                       qj - joint position (NumDoF x 1)
%
%   Returns :   J - Jacobian (6 x (NumDoF + 6))
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 1 
            J = wholeBodyModel('jacobian',varargin{1});
        case 2
            J = wholeBodyModel('jacobian',varargin{1},varargin{2});
        otherwise
            disp('Incorrect number of arguments, check docs');
    end
end
