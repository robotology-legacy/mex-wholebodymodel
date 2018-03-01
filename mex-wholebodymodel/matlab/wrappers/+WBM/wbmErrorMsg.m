% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU project CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
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

classdef wbmErrorMsg
    % :class:`!wbmErrorMsg` is a *utility class* with a set of constant member
    % variables of predefined error messages for the WBM-Library.
    properties(Constant)
        DIM_MISMATCH      = 'Mismatching dimensions!';
        EMPTY_ARRAY       = 'The array is empty!';
        EMPTY_DATA_PTS    = 'The set of data points is empty!';
        EMPTY_DATA_TYPE   = 'Empty data type!';
        EMPTY_STRING      = 'Empty string!';
        EMPTY_VECTOR      = 'Empty vector(s)!';
        FILE_NOT_EXIST    = 'File does not exist on given path!';
        IDX_OUT_OF_BOUNDS = 'Index value out of bounds!';
        LIST_NOT_SORTED   = 'List not in ascending order!';
        LNK_NOT_DEF       = 'The link is not defined!';
        LNK_NOT_IN_LIST   = 'The searched link is not in the list!';
        MAX_JOINT_LIMIT   = 'Maximum joint number exceeded!';
        MAX_NUM_LIMIT     = 'Value exceeds the maximum number!';
        NAME_NOT_EXIST    = 'The name does not exist in the list!';
        NOT_HOMOG_MAT     = 'Matrix is not homogeneous!';
        NO_LNK_IN_CTC     = 'At least one link must be in contact with the ground or object!';
        OBJ_NOT_OBSTACLE  = 'The object is not defined as an obstacle!';
        SINGULAR_MAT      = 'Singular matrix!';
        STRING_MISMATCH   = 'String mismatch!';
        UDEF_QUAT_VEC     = 'The quaternion is undefined!';
        UDEF_ROT_MAT      = 'The rotation matrix is undefined!';
        UNKNOWN_AXIS_SEQ  = 'Unknown axis sequence!';
        UNKNOWN_CTC_MODEL = 'Unknown contact model!';
        UNKNOWN_EXC       = 'Unknown exception!';
        UNKNOWN_JNT_TYPE  = 'Unknown joint type!';
        UNKNOWN_LNK_NAME  = 'Unknown link name!';
        VALUE_IS_INIT     = 'The value is already initialized!';
        VALUE_LTE_ZERO    = 'The value must be greater than zero!';
        VALUE_LT_ZERO     = 'The value must be positive!';
        VAL_OUT_OF_BOUNDS = 'Value(s) out of bounds!';
        WRONG_ARR_DIM     = 'Wrong array dimension!';
        WRONG_ARR_SIZE    = 'Wrong array size!';
        WRONG_DATA_TYPE   = 'Wrong data type!';
        WRONG_MAT_DIM     = 'Wrong matrix dimension!';
        WRONG_NARGIN      = 'Incorrect number of input arguments!';
        WRONG_NARGOUT     = 'Incorrect number of output arguments!';
        WRONG_VEC_DIM     = 'Wrong vector dimension!';
        WRONG_VEC_LEN     = 'Wrong vector length!';
    end
end
