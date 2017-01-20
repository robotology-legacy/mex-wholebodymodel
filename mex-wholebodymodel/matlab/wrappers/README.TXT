The complete refactoring of the mexWholeBodyModel subroutine for Matlab has caused a lot of changes inside in the C++ code as also outside in the wrapper functions. During this process and the code-style unification, a lot of variable and function names were renamed with more meaningful names.

Because of this reason, and also to be backward compatible with older or current running projects, the folder "old" in the wrapper directory should be added to the Matlab-path.

For all new projects it is recommended to use the new wrapper-functions or the object-oriented WBM-classes.


Author: Martin Neururer (martin.neururer@gmail.com)
Genova, Jan 2017
