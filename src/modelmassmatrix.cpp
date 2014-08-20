/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 *
 * The development of this software was supported by the FP7 EU projects
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

//global includes

//library includes 
#include <wbi/iWholeBodyModel.h>
#include <wbiIcub/icubWholeBodyModel.h>

//local includes
#include "modelmassmatrix.h"

using namespace mexWBIComponent;

ModelMassMatrix * ModelMassMatrix::modelMassMatrix; 

ModelMassMatrix::ModelMassMatrix(): ModelComponent(1,0,1)
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

ModelMassMatrix * ModelMassMatrix::getInstance() 
{
  if(modelMassMatrix == NULL)
  {
    modelMassMatrix = new ModelMassMatrix;
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

bool ModelMassMatrix::computeFast(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelMassMatrix \n");
#endif
  robotModel = modelState->robotModel();
  qj = modelState->qj();
  xB = modelState->rootRotoTrans();  
  
  if(!robotModel->computeMassMatrix(qj,xB,massMatrix))
  {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI MassMatrix call");
  }

  
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
  robotModel = modelState->robotModel();
  qj = mxGetPr(prhs[1]);
#ifdef DEBUG
  mexPrintf("qj received \n");

  for(int i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif  
  //int LINK_FOOT_WRF;
  
  
  robotModel->computeH(qj,wbi::Frame(),ROBOT_BASE_FRAME_LINK, H_base_wrfLink);
  
  H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
  xB.set4x4Matrix (H_w2b.data());
  
  if(massMatrix != NULL)
  {
     if(!robotModel->computeMassMatrix(qj,xB,massMatrix))
     {
	mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI MassMatrix call");
     }
  }
//   mxFree(q);
  return(true);  
}




















