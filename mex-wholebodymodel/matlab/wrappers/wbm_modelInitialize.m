function wbm_modelInitialize(varargin)
    % WBM_MODELINITIALIZE initializes the whole-body model of the robot.
    %
    % Input Arguments:
    %
    %   Normal mode:
    %       robot_name -- String matching name of the robot model.
    %
    %                     Note: The URDF-file of the given robot model must
    %                     exist in the directory of the yarpWholeBodyInterface
    %                     or CoDyCo-Superbuild.
    %
    %   Optimized mode:  No arguments.
    %
    % See also: WBM_MODELINITIALIZEFROMURDF

    % Copyright (C) 2014-2017 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    %
    % This function is part of the Whole-Body Model Library for Matlab (WBML).
    %
    % Permission is granted to copy, distribute, and/or modify the WBM-Library
    % under the terms of the GNU Lesser General Public License, Version 2.1
    % or any later version published by the Free Software Foundation.
    %
    % The WBM-Library is distributed in the hope that it will be useful,
    % but WITHOUT ANY WARRANTY; without even the implied warranty of
    % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    % GNU Lesser General Public License for more details.
    %
    % A copy of the GNU Lesser General Public License can be found along
    % with the WBML. If not, see <http://www.gnu.org/licenses/>.
    switch nargin
        case 0
            mexWholeBodyModel('model-initialize');
        case 1
            mexWholeBodyModel('model-initialize', varargin{1,1});
        otherwise
            wbm_narginError('wbm_modelInitialize');
    end
end
