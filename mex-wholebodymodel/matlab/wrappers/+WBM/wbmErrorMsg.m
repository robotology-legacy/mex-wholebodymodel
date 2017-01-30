classdef wbmErrorMsg
    properties(Constant)
        DIM_MISMATCH      = 'Mismatching dimensions!';
        EMPTY_ARRAY       = 'The array is empty!';
        EMPTY_DATA_TYPE   = 'Empty data type!';
        EMPTY_STRING      = 'Empty string!';
        EMPTY_VECTOR      = 'Empty vector(s)!';
        FILE_NOT_EXIST    = 'File does not exist on given path!';
        IDX_OUT_OF_BOUNDS = 'Index value out of bounds!';
        MAX_JOINT_LIMIT   = 'Maximum joint number exceeded!';
        MAX_NUM_LIMIT     = 'Value exceeds the maximum number!';
        NAME_NOT_EXIST    = 'The name does not exist in the list!'
        NOT_HOMOG_MAT     = 'Matrix is not homogeneous!';
        SINGULAR_MAT      = 'Singular matrix!';
        STRING_MISMATCH   = 'String mismatch!';
        UNKNOWN_AXIS_SEQ  = 'Unknown axis sequence!';
        UNKNOWN_EXC       = 'Unknown exception!';
        UNKNOWN_JNT_TYPE  = 'Unknown joint type!';
        UNKNOWN_LNK_NAME  = 'Unknown link name!';
        VALUE_IS_INIT     = 'The value is already initialized!';
        VALUE_LTE_ZERO    = 'The value must be greater than zero!';
        WRONG_ARG         = 'Wrong number of input arguments!';
        WRONG_ARR_SIZE    = 'Wrong array size!';
        WRONG_DATA_TYPE   = 'Wrong data type!';
        WRONG_MAT_DIM     = 'Wrong matrix dimension!';
        WRONG_VEC_DIM     = 'Wrong vector dimension!';
        WRONG_VEC_SIZE    = 'Wrong vector size!';
    end
end
