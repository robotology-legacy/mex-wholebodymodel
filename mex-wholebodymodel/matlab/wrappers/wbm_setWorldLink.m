function []  = wbm_setWorldLink( varargin )
%WBM_SETWORLDFRAME Sets the world frame at a given rototranslation from the
%from a chosen reference link. At a given joint configuration, the
%rototranslation from the root link to the world can then be computed to
%aid optimised computation of all other dynamics components
%
%   Arguments : 
%       Optimised Mode : R - rotation from reference link to world frame (3 x 3)
%                        p - translation from reference link to world frame (3 x 1) 
%                        (link name used is the previously set or default)
%       Normal Mode :  link_name - string matching URDF name of the link (frame)
%                      R - rotation from reference link to world frame (3 x 3)
%                      p - translation from reference link to world frame (3 x 1) 
%                      g - gravity vector in the world frame (3 x 1)
%   Returns :   None
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    switch(nargin)
        case 2
            wholeBodyModel('set-world-link',reshape(varargin{1},[],1), varargin{2});
        case 4
            wholeBodyModel('set-world-link',varargin{1}, reshape(varargin{2},[],1), varargin{3}, varargin{4});
        otherwise
             disp('setWorldLink : Incorrect number of arguments, check docs'); 
    end
end

