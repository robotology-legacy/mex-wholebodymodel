function [ J ] = wbm_jacobian( varargin )
%WMB_JACOBIAN computes the Jacobian to a desired link (frame) at a given
%joint configuration
%velocity vxb
%   Arguments : 
%       Optimised Mode :  link_name - string matching URDF name of the link (frame)
%       Normal Mode :  R - rotation from rootLink to world frame (3 x 3)
%                      p - translation from rootLink to world frame (3 x 1) 
%                      qj - joint position (NumDoF x 1)
%                      link_name - string matching URDF name of the link (frame)
%
%   Returns :   J - Jacobian (6 x (NumDoF + 6))
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 1 
            J = wholeBodyModel('jacobian',varargin{1});
        case 4
            J = wholeBodyModel('jacobian',reshape(varargin{1},[],1), varargin{2},varargin{3},varargin{4});
        otherwise
            disp('jacobian : Incorrect number of arguments, check docs');
    end
end
