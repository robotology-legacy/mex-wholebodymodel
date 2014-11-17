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

// library includes
#include <wbi/iWholeBodyModel.h>
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>
//#include <wbiIcub/icubWholeBodyModel.h>
//#include <wbiIcub/wbiIcubUtil.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>

// project includes
#include "modelcomponent.h"
#include "modeldjdq.h"

using namespace mexWBIComponent;

ModelDjDq * ModelDjDq::modelDjDq;

ModelDjDq::ModelDjDq(void) : ModelComponent(4,1,1)
{
#ifdef DEBUG
  mexPrintf("ModelGeneralisedBiasForces constructed\n");
#endif
}

ModelDjDq::~ModelDjDq()
{
  
}

ModelDjDq * ModelDjDq::getInstance(void)
{
  if(modelDjDq == NULL)
  {
    modelDjDq = new ModelDjDq;
  }
  return(modelDjDq);
}

bool ModelDjDq::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelDjDq\n");
#endif
//   bool returnVal = false;
    if(nlhs!=1)
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs", "one output argument required for computing ModelDjDq.");
    }

    plhs[0] =  mxCreateNumericMatrix(6, 1, mxDOUBLE_CLASS, mxREAL);//

    Djdq = mxGetPr(plhs[0]);
    return(true);
}

bool ModelDjDq::compute(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelDjDq\n");
#endif

  processArguments(nrhs,prhs);

  return(true);
}

bool ModelDjDq::computeFast(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
    mexPrintf("Trying to compute ModelDjDq\n");
#endif
   
  if(!mxIsChar(prhs[1]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }

  robotModel = modelState->robotModel();
  qj = modelState->qj();
  qjDot = modelState->qjDot();
  vb = modelState->vb();
  xB = modelState->rootRotoTrans();
  refLink = mxArrayToString(prhs[1]);
  int refLinkID;
  //robotModel->getLinkId (refLink, refLinkID);
  std::string com("com");
  
  if(com.compare(refLink)==0)
  {
    refLinkID = -1;
  }
  else
  {
    //robotModel->getLinkId (refLink, refLinkID);
    robotModel->getFrameList().idToIndex(refLink,refLinkID);
  }
  
  if(!robotModel->computeDJdq(qj,xB,qjDot,vb,refLinkID,Djdq))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI DJDq call");
  }

  return(true);
}

/*
bool ModelDjDq::display(int nrhs, const mxArray* prhs[])
{
  mexPrintf("Trying to display ModelDjDq:\n");
  return(true);
}*/
bool ModelDjDq::processArguments(int nrhs, const mxArray* prhs[])
{
//   if(nrhs<5)
//   {
//      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Atleast 5 input arguments required for modelDjDq");
//   }
  

  
  if(mxGetM(prhs[1]) != numDof || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != numDof || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 6 || mxGetN(prhs[3]) != 1 || !(mxIsChar(prhs[4])))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions / inputs");
  }
  robotModel = modelState->robotModel();
  qj = mxGetPr(prhs[1]);
  qjDot = mxGetPr(prhs[2]);
  vb = mxGetPr(prhs[3]);
  refLink = mxArrayToString(prhs[4]);
  
#ifdef DEBUG
  mexPrintf("qj received \n");

  for(int i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif  

  xB = computeRootWorldRotoTranslation(qj);
  
  if(Djdq != NULL)
  {
    int refLinkID;
    //robotModel->getLinkId(refLink,refLinkID);
    std::string com("com");
  
    if(com.compare(refLink)==0)
    {
      refLinkID = -1;
    }
    else
    {
      //robotModel->getLinkId (refLink, refLinkID);
      robotModel->getFrameList().idToIndex(refLink,refLinkID);
    }
    if(!robotModel->computeDJdq(qj,xB,qjDot,vb,refLinkID,Djdq))
    {
       mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI DJDq call");
    }
  }
}

