/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2014  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

//global includes

//library includes 
#include <wbi/iWholeBodyModel.h>
#include <wbiIcub/icubWholeBodyModel.h>

//local includes
#include "modelmassmatrix.h"

using namespace mexWBIComponent;

ModelMassMatrix * ModelMassMatrix::modelMassMatrix; 

ModelMassMatrix::ModelMassMatrix(wbi::iWholeBodyModel *m): ModelComponent(m,1,1)
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix constructed \n");
#endif
}

ModelMassMatrix::~ModelMassMatrix()
{

}

bool ModelMassMatrix::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelMassMatrix\n");
#endif
  bool returnVal = false;
//   if(nlhs!=1)
//   {
//      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumOutputs","1 output argument required for massmatrix");
//   }
  plhs[0]=mxCreateDoubleMatrix(numDof+6,numDof+6, mxREAL);
  
  massMatrix = mxGetPr(plhs[0]);
  returnVal = true;
  return(returnVal);
}

ModelMassMatrix * ModelMassMatrix::getInstance(wbi::iWholeBodyModel *m) 
{
  if(modelMassMatrix == NULL)
  {
    modelMassMatrix = new ModelMassMatrix(m);
  }
  return(modelMassMatrix);
}

/*
bool ModelMassMatrix::display(int nrhs, const mxArray * prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to display ModelMassMatrix \n");
#endif
  bool processRet = processArguments(nrhs,prhs);
  //processArguments(nrhs,prhs);
  double *mm = new double((numDof+6) *(numDof+6));
  
  //robotModel->computeMassMatrix(modelState->q(),modelState->baseFrame(),mm);
#ifdef DEBUG
  mexPrintf("Trying to display ModelMassMatrix : call from wbi returned\n");
#endif
  //double qState[numDof];
  for( int i = 0; i<numDof+6; i++)
  {
    for(int j = 0; j<numDof+6; j++)
    {
	mm[i+j*numDof] = massMatrix[i+j*numDof];
	//mm[i][j] = 0.5 + 0.1*i*j;
#ifdef DEBUG
	mexPrintf("%f ",mm[i+j*numDof]);
#endif
	
      //mexPrintf("%f ",massMatrix[i+(j*numDof)]);
    }
#ifdef DEBUG
    mexPrintf("\n ");
#endif
    
  }
  
  delete(mm);
    //robotModel->computeMassMatrix()
  return(true);
}*/

bool ModelMassMatrix::compute(int nrhs, const mxArray * prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelMassMatrix \n");
#endif
  processArguments(nrhs,prhs);
#ifdef DEBUG
  mexPrintf("ModelMassMatrix computed\n");
#endif
  return(true);
}


bool ModelMassMatrix::processArguments(int nrhs, const mxArray * prhs[])
{
//   if(nrhs<2)
//   {
//      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Atleast two input arguments required for mass-matrix");
//   }
  
  if(mxGetM(prhs[1]) != numDof || mxGetN(prhs[1]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions");
  }
    
  q = mxGetPr(prhs[1]);
#ifdef DEBUG
  mexPrintf("q received \n");

  for(int i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",q[i]);
  }
#endif  
  //int LINK_FOOT_WRF;
  
  
  robotModel->computeH(q,wbi::Frame(),ROBOT_BASE_FRAME_LINK, H_base_wrfLink);
  
  H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
  xB.set4x4Matrix (H_w2b.data());
  
  if(massMatrix != NULL)
  {
     robotModel->computeMassMatrix(q,xB,massMatrix);
  }
//   mxFree(q);
  return(true);  
}




















