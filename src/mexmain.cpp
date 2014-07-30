#include <iostream> 
#include <stdio.h>
#include <mex.h>


// Local includes
#include <modelcomponent.h>
#include <componentmanager.h>
//#include <modeljointlimits.h>

// namespaces
using namespace mexWBIComponent;
static ComponentManager *componentManager;

void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
  componentManager = ComponentManager::getInstance();
#ifdef DEBUG
  mexPrintf("starting to process function\n");
#endif
// Check for proper number of input and output arguments 
    if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Atleast one input argument required.");
    }
    if (nlhs > 2){
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:maxlhs","Too many output arguments.");
    }

// Check to be sure input is of type char 
    if (!(mxIsChar(prhs[0]))){
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Input must be of type string.\n.");
    }

//       if(componentManager->parseMexArguments(nrhs,prhs))
//       {
// 	if(nlhs > 0)
// 	{
// 	  componentManager->allocateReturnSpace(nlhs,plhs);//  
// 	  componentManager->computeComponent(nlhs,plhs,nrhs,prhs);// == true)
// 	}
// 	else
// 	{
// 	  componentManager->displayComponent(nlhs,plhs,nrhs,prhs);
// 	}
//       }
  componentManager->processFunctionCall(nlhs,plhs,nrhs,prhs);
 //componentManager->parseMexArguments(nrhs,prhs);
}



// #include <stdio.h>
// #include "mex.h"
// void mexFunction(int nlhs, mxArray *plhs[],
//                  int nrhs, const mxArray *prhs[])
// {
//     double *input, *output;
//     int i, j, n_rows, n_cols;
//     double avg;
//     
//     mxArray **temp;
// 
//     if ((nrhs != 1) || (nlhs > 1)) {
//         printf("Error! Expecting exactly 1 rhs and up to 1 lhs argument!\n");
//         return;
//     }
//         
//     input = mxGetPr(prhs[0]);
//     n_rows = mxGetM(prhs[0]);
//     n_cols = mxGetN(prhs[0]);
// 
//     if (nlhs == 1) {
//     //    plhs[0] = mxCreateDoubleMatrix(n_rows, 1, mxREAL);
//       temp[0] = mxCreateDoubleMatrix(n_rows,1,mxREAL);
//       plhs[0]=temp[0];
//       //output = mxGetPr(plhs[0]);
//       output = mxGetPr(temp[0]);
//     }
//         
//     for(i=0; i<n_rows; i++) {
//         avg = 0.;
//         for(j=0; j<n_cols; j++)
//             avg += input[(i*n_cols)+j];
//         avg /= (double)n_cols;
// 
//         if (nlhs == 1)
//             output[i] = avg;
//     }
//     
//    // plhs = temp;
// } /* end mexFunction */