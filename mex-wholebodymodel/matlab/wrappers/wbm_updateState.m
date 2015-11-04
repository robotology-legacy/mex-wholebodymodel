function []  = wbm_updateState( varargin )
%WBM_UPDATESTATE Updates the robot state, i.e. joint angles, joint
%velocities, floating base velocity.
%
%   Arguments : 
%       Optimised Mode : Does not exist
%       Normal Mode : qj - joint angles (numDoF x 1)
%                     qjDot - joint velocity (numDoF x 1)
%                     vwb - floating base velocity (6 x 1)
%   Returns :   None
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 3
            wholeBodyModel('update-state',varargin{1}, varargin{2}, varargin{3});
        otherwise
             disp('updateState : Incorrect number of arguments, check docs'); 
    end
end

