
#include <iostream>
#include "icubWBModelTester.h"

#include <mex.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>


//static 
IcubWBModelTester *icubWBIModelTest;

/*
 * 
 * nlhs
 *Number of expected output mxArrays
 *plhs
 *Array of pointers to the expected output mxArrays
 *nrhs
 *Number of input mxArrays
 *prhs
*/


void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{

//int main(int argc, char **argv) {
  //std::cout << "Starting...nlhs:"<<nlhs<<"plhs0: "<<mxArrayToString(prhs[0])<<"plhs1: "<<mxArrayToString(prhs[1])<<"nrhs: "<<nrhs<<"prhs: "<<(char *)prhs<<"\n";  
  int time = 1;
  
    if (nrhs != 2) {
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs", "Two input arguments required.");
    }
    if (nlhs > 2){
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:maxlhs","Too many output arguments.");
    }
   if (!(mxIsChar(prhs[0])) && !(mxIsChar(prhs[1]))){
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Inputs must be of type string.\n.");
    }else
    {
        icubWBIModelTest = new IcubWBModelTester;
    }
//   while(time--)
//   {
//     std::cout <<" Yo, itsaa meeya, YARP!" << std::endl;
    
    icubWBIModelTest->runTest(0);
//   }
  
    mxArray *minAngles;
    mxArray *maxAngles;
    
//     mxArray *mxCreateDoubleMatrix(mwSize m, mwSize n, mxComplexity ComplexFlag);
    
 delete(icubWBIModelTest);

}
