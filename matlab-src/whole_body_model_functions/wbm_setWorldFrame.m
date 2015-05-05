function []  = wbm_setWorldFrame( varargin )
%WBM_SETWORLDFRAME Sets the world frame at a given rototranslation from the
%from a chosen reference link. At a given joint configuration, the
%rototranslation from the root link to the world can then be computed to
%aid optimised computation of all other dynamics components
%
%   Arguments : 
%       Optimised Mode : Does not exist
%       Normal Mode :  R - rotation from rootLink to world frame (3 x 3)
%                      p - translation from rootLink to world frame (3 x 1) 
%                      g - gravity vector in world frame
%   Returns :   None
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 3
            wholeBodyModel('set-world-frame',reshape(varargin{1},[],1), varargin{2}, varargin{3});
        otherwise
             disp('setWorldFrame : Incorrect number of arguments, check docs'); 
    end
end

