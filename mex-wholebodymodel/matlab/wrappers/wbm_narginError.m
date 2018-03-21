function wbm_narginError(func_name)
    % WBM_NARGINERROR throws an error and displays an error message that the NARGIN of
    % the specified function is incorrect.
    %
    % Input Arguments:
    %    func_name -- The name of the current function (string).

    % Copyright (C) 2017 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
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

    errmsg = sprintf('%s Check the documentations!', WBM.wbmErrorMsg.WRONG_NARGIN);
    error('%s: %s\n', func_name, errmsg);
end
