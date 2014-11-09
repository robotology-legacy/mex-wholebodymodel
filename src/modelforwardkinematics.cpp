/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 *  Authors: Naveen Kuppuswamy
 *  email: naveen.kuppuswamy@iit.it
 * 
 *  The development of this software was supported by the FP7 EU projects
 *  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 *  http://www.codyco.eu
 * 
 *  Permission is granted to copy, distribute, and/or modify this program
 *  under the terms of the GNU General Public License, version 2 or any
 *  later version published by the Free Software Foundation.
 * 
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details
 *  
 * 
 */

// global includes
#include<string.h>
// library includes

//local includes


#include "modelforwardkinematics.h"

using namespace mexWBIComponent;
ModelForwardKinematics * ModelForwardKinematics::modelForwardKinematics;

ModelForwardKinematics::ModelForwardKinematics() : ModelComponent(2,1,1)
{

}

ModelForwardKinematics::~ModelForwardKinematics()
{

}

bool ModelForwardKinematics::allocateReturnSpace(int nlhs, mxArray* plhs[])
{

  #ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelForwardKinematics\n");
#endif
  
  bool returnVal = false;

  plhs[0]=mxCreateDoubleMatrix(7,1, mxREAL);
  xT = mxGetPr(plhs[0]);
  returnVal = true;
  return(returnVal);
}
bool ModelForwardKinematics::compute(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Tring to compute in ModelForwardKinematics");
#endif
  processArguments(nrhs,prhs);
  return(true);
  
}

bool ModelForwardKinematics::processArguments(int nrhs, const mxArray* prhs[])
{
//   if(nrhs<3)
//   {
//      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Atleast three input arguments required for ModelJacobian");
//   }
  
  if(mxGetM(prhs[1]) != numDof || mxGetN(prhs[1]) != 1 || !mxIsChar(prhs[2]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
  robotModel = modelState->robotModel(); 
  qj = mxGetPr(prhs[1]);
  refLink = mxArrayToString(prhs[2]);
#ifdef DEBUG
  mexPrintf("qj received \n");

  for(int i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif  
  
  xB = computeRootWorldRotoTranslation(qj);
  
  if(xT != NULL)
  {
    int refLinkID;
    std::string com("com");
  
    if(com.compare(refLink)==0)
    {
      refLinkID = -1;
    }
    else
    {
      robotModel->getLinkId (refLink, refLinkID);
    }
    
     //robotModel->computeMassMatrix(q,xB,massMatrix);
    //if(!(robotModel->computeJacobian(q,xB,refLinkID,j)))
    if(!(robotModel->forwardKinematics(qj,xB,refLinkID,xT)))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the forwardKinematics call");
    }
  }
//   mxFree(q);
  return(true);  
}

bool ModelForwardKinematics::computeFast(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to fastCompute ModelJacobian \n");
#endif
  
  if(!mxIsChar(prhs[1]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
  robotModel = modelState->robotModel(); 
  qj = modelState->qj();
  xB = modelState->rootRotoTrans();
  refLink = mxArrayToString(prhs[1]);
  int refLinkID;
  
  std::string com("com");
  
  if(com.compare(refLink)==0)
  {
    refLinkID = -1;
  }
  else
  {
    robotModel->getLinkId (refLink, refLinkID);
  }

  if(!(robotModel->forwardKinematics(qj,xB,refLinkID,xT)))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the forwardKinematics call");
  }
#ifdef DEBUG
  mexPrintf("ModelJacobian fastComputed\n");
#endif
  return(true);
}

ModelForwardKinematics* ModelForwardKinematics::getInstance()
{
  if(modelForwardKinematics == NULL)
  {
    modelForwardKinematics = new ModelForwardKinematics();
  }
  return(modelForwardKinematics);

}

