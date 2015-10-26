function [qj,xTb,qjDot,vb]  = wbm_getState( varargin )
%WBM_GETSTATE Obtains the currently stored state (joint angles, floating base joint
%velocities, floating base velocity)
%   Arguments :
%       Optimised Mode : No arguments
%       Normal Mode :  No arguments
%   Returns :   qj - joint angles (NumDoF x 1)
%               xTb - base rototranslation (7 x 1) with 3 for position of frame and 4 for orientation quaternion
%                      (return is already flipped so that quaternion organised as real
%                      followed by imaginary)
%               dqj - joint velocities (NumDoF x 1)
%               vxb - floating base velocity (6 x 1)
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 0
            [qj,xTbT,qjDot,vb]  = wholeBodyModel('get-state');
            xTb = xTbT;
        otherwise
             disp('getState : Incorrect number of arguments, check docs');
    end
end

