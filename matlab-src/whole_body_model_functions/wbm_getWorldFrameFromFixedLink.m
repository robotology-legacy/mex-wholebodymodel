function [ world_R_base,world_p_base ] = wbm_getWorldFrameFromFixedLink( varargin )
%WBM_GETWORLDFRAMEFROMFIXEDLINK returns the floating base position for a
%given reference link name and joint configuration
%
%   Arguments :
%       Optimised Mode : link_name - string matching URDF name of the link (frame)
%       Normal Mode : link_name - string matching URDF name of the link (frame)
%                      qj - joint configuration ( (NumDoF x 1))
%   Returns : R - rotation from rootLink to world frame (3 x 3)
%             p - translation from rootLink to world frame (3 x 1) 
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 1 
            qT = wbm_forwardKinematics(varargin{1});
            p = zeros(3,1);
            [base_p_reference,base_R_reference] = frame2posrot(qT);
            world_p_base = -base_p_reference;
            world_R_base = base_R_reference';
        case 2 
            qT = wbm_forwardKinematics(eye(3),zeros(3,1),varargin{2},varargin{1});
            p = zeros(3,1);
            [base_p_reference,base_R_reference] = frame2posrot(qT);
            world_p_base = -base_p_reference;
            world_R_base = base_R_reference';
        otherwise
            disp('getWorldFrameFromFixedLink : Incorrect number of arguments, check docs');
    end
            
end

