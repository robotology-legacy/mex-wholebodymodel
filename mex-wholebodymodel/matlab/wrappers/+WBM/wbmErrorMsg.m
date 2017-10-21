classdef wbmErrorMsg
    properties(Constant)
        DIM_MISMATCH      = 'Mismatching dimensions!';
        EMPTY_ARRAY       = 'The array is empty!';
        EMPTY_DATA_TYPE   = 'Empty data type!';
        EMPTY_STRING      = 'Empty string!';
        EMPTY_VECTOR      = 'Empty vector(s)!';
        FILE_NOT_EXIST    = 'File does not exist on given path!';
        IDX_OUT_OF_BOUNDS = 'Index value out of bounds!';
        LIST_NOT_SORTED   = 'List not in ascending order!';
        LNK_NOT_IN_LIST   = 'The searched link is not in the list!';
        MAX_JOINT_LIMIT   = 'Maximum joint number exceeded!';
        MAX_NUM_LIMIT     = 'Value exceeds the maximum number!';
        NAME_NOT_EXIST    = 'The name does not exist in the list!';
        NOT_HOMOG_MAT     = 'Matrix is not homogeneous!';
        NO_LNK_IN_CTC     = 'At least one link must be in contact with the ground or object!';
        OBJ_NOT_OBSTACLE  = 'The object is not defined as an obstacle!';
        SINGULAR_MAT      = 'Singular matrix!';
        STRING_MISMATCH   = 'String mismatch!';
        UNKNOWN_AXIS_SEQ  = 'Unknown axis sequence!';
        UNKNOWN_CTC_MODEL = 'Unknown contact model!';
        UNKNOWN_EXC       = 'Unknown exception!';
        UNKNOWN_JNT_TYPE  = 'Unknown joint type!';
        UNKNOWN_LNK_NAME  = 'Unknown link name!';
        VALUE_IS_INIT     = 'The value is already initialized!';
        VALUE_LTE_ZERO    = 'The value must be greater than zero!';
        VALUE_LT_ZERO     = 'The value must be positive!';
        VAL_OUT_OF_BOUNDS = 'Value(s) out of bounds!';
        WRONG_ARR_SIZE    = 'Wrong array size!';
        WRONG_DATA_TYPE   = 'Wrong data type!';
        WRONG_MAT_DIM     = 'Wrong matrix dimension!';
        WRONG_NARGIN      = 'Incorrect number of input arguments!';
        WRONG_NARGOUT     = 'Incorrect number of output arguments!';
        WRONG_VEC_DIM     = 'Wrong vector dimension!';
        WRONG_VEC_LEN     = 'Wrong vector length!';
    end
end
