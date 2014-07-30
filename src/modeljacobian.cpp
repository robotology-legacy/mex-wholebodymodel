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

#include "modeljacobian.h"
#include <wbi/iWholeBodyModel.h>
using namespace mexWBIComponent;

ModelJacobian * ModelJacobian::modelJacobian; 

ModelJacobian::ModelJacobian(wbi::iWholeBodyModel *m): ModelComponent(m),numReturnArguments(1)
{
  mexPrintf("ModelJacobian constructed \n");
//   numDof = robotModel->getDoFs();
}

ModelJacobian::~ModelJacobian()
{

}

//  static ModelMassMatrix* getInstance(wbi::iWholeBodyModel *);
//   virtual int numReturns();
//   virtual void display();
//   virtual void compute();
//   
//  virtual bool allocateReturnSpace(int, mxArray *[]);

bool ModelJacobian::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelMassMatrix\n");
#endif
  bool returnVal = false;
  if(nlhs!=1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumOutputs","1 output argument required for massmatrix");
  }
//   if(nlhs>=1)
//   {
//     
    //plhs[0] =  mxCreateNumericMatrix(modelState->dof()+6, modelState->dof()+6, mxDOUBLE_CLASS, mxREAL);//
    plhs[0]=mxCreateDoubleMatrix(6,numDof+6, mxREAL);
    //plhs[1]=mxCreateDoubleMatrix(numDof,1, mxREAL);
  
    j = mxGetPr(plhs[0]);
   // jointUpperLimit = mxGetPr(plhs[1]);
    returnVal = true;
//   }
  
  return(returnVal);
}

const int ModelJacobian::numReturns()
{
  return(numReturnArguments);
}

ModelJacobian * ModelJacobian::getInstance(wbi::iWholeBodyModel *m) 
{
  if(modelJacobian == NULL)
  {
    modelJacobian = new ModelJacobian(m);
  }
  return(modelJacobian);
}


bool ModelJacobian::display(int nrhs, const mxArray * prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to display ModelMassMatrix \n");
#endif
  bool processRet = processArguments(nrhs,prhs);
  //processArguments(nrhs,prhs);
  double *mm = new double((6) *(numDof));
  
  //robotModel->computeMassMatrix(modelState->q(),modelState->baseFrame(),mm);
#ifdef DEBUG
  mexPrintf("Trying to display ModelMassMatrix : call from wbi returned\n");
#endif
  double qState[numDof];
  for( int i = 0; i<6; i++)
  {
    for(int k = 0; k<numDof+6; k++)
    {
	mm[i+k*numDof] = j[i+k*numDof];
	//mm[i][j] = 0.5 + 0.1*i*j;
#ifdef DEBUG
	mexPrintf("%f ",mm[i+k*numDof]);
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
}

bool ModelJacobian::compute(int nrhs, const mxArray * prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelJacobian \n");
#endif
  processArguments(nrhs,prhs);
#ifdef DEBUG
  mexPrintf("ModelJacobian computed\n");
#endif
  return(true);
}



bool ModelJacobian::processArguments(int nrhs, const mxArray * prhs[])
{
  if(nrhs<3)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Atleast three input arguments required for ModelJacobian");
  }
  
  if(mxGetM(prhs[1]) != numDof || mxGetN(prhs[1]) != 1 || !mxIsChar(prhs[2]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
    
  q = mxGetPr(prhs[1]);
  refLink = mxArrayToString(prhs[2]);
#ifdef DEBUG
  mexPrintf("q received \n");

  for(int i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",q[i]);
  }
#endif  
  Eigen::Matrix4d H_w2b;
  int LINK_FOOT_WRF;
  wbi::Frame H_base_wrfLink,xB;
  robotModel->getLinkId ("l_sole", LINK_FOOT_WRF);
  robotModel->computeH(q,wbi::Frame(),LINK_FOOT_WRF, H_base_wrfLink);
  
  H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
  xB.set4x4Matrix (H_w2b.data());
  
  if(j != NULL)
  {
    int refLinkID;
    robotModel->getLinkId (refLink, refLinkID);
     //robotModel->computeMassMatrix(q,xB,massMatrix);
    if(!(robotModel->computeJacobian(q,xB,refLinkID,j)))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the jacobian call");
    }
  }
//   mxFree(q);
  return(true);  
}




















