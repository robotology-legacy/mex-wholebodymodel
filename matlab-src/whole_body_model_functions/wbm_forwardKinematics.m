    `function [p] = wbm_forwardKinematics(varargin)
%WBM_FORWARDKINEMATICS computes the forward kinematics rototranslation to a specified link in the current joint
%   configuration. 
%   Arguments :
%       Optimised Mode : link_name - string matching URDF name of the link (frame)
%       Normal Mode : qj - joint position (NumDoF x 1)
%                     link_name - string matching URDF name of the link
%                     (frame)
%   Returns :   p - 7 dimensional vector of the rototranslation of link
%   frame. First 3 elements are the position, next 4 are the orientation in
%   quaternions
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014


    switch(nargin)
        case 1
            p = wholeBodyModel('forward-kinematics',varargin{1});
        case 2
            p = wholeBodyModel('forward-kinematics',varargin{1},varargin{2});
        otherwise
            disp('forwardKinematics : Incorrect number of arguments, check docs');        
    end
    
end