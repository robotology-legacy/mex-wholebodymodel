function [ world_R_base,world_p_base ] = wbm_getWorldFrameFromFixedLink( varargin )
%WBM_GETWORLDFRAMEFROMFIXEDLINK returns the floating base position wrt to a
%world frame that is intentionally set at a specified link frame. The
%returned floating base position and rotation is obtained from  forward kinematics 
%wrt the specified link frame
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
            [world_R_base, world_p_base]= computeNewWorldToBase(varargin{1});                
        case 2 
            [world_R_base, world_p_base]= computeNewWorldToBase(varargin{1},varargin{2});
        otherwise
            disp('getWorldFrameFromFixedLink : Incorrect number of arguments, check docs');
    end
            
end


function [world_R_base, world_p_base] = computeNewWorldToBase(varargin)
    
    [~,oldWorld_qH_base,~,~]  = wbm_getState();
    [oldWorld_p_base,oldWorld_R_base] = frame2posrot(oldWorld_qH_base);
    oldWorld_H_base = [oldWorld_R_base oldWorld_p_base; zeros(1,3) 1];

    
    if(nargin == 1)
        [oldWorld_qH_referenceLink] = wbm_forwardKinematics(varargin{1});
    else
        [oldWorld_qH_referenceLink] = wbm_forwardKinematics(oldWorld_R_base,oldWorld_p_base,varargin{2},varargin{1});
    end

     [oldWorld_p_referenceLink,oldWorld_R_referenceLink] = frame2posrot(oldWorld_qH_referenceLink);
                
     oldWorld_H_referenceLink = [oldWorld_R_referenceLink oldWorld_p_referenceLink; zeros(1,3) 1];
     newWorld_H_base = oldWorld_H_referenceLink \ oldWorld_H_base;
            
     world_p_base = newWorld_H_base(1:3,4);
     world_R_base = newWorld_H_base(1:3,1:3);
end
