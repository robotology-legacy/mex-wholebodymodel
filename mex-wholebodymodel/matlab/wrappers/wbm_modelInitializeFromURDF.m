function wbm_modelInitializeFromURDF(varargin)
    % WBM_MODELINITIALIZEFROMURDF initializes the whole-body model of a YARP-based robot
    % with a specific URDF-file.
    %
    % Input Arguments:
    %   urdf_file_name -- String with the full path to the URDF-file to read.
    %
    % See also: WBM_MODELINITIALIZE

    % Copyright (C) 2014-2017 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
    % Author: Silvio Traversaro (silvio.traversaro@iit.it); Genova, Nov 2015
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
        case 1
            mexWholeBodyModel('model-initialize-urdf', varargin{1,1});
        otherwise
            wbm_narginError('wbm_modelInitializeFromURDF');
    end
end
