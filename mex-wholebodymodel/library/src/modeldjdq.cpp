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

ModelDjDq::ModelDjDq(void) : ModelComponent(6,1,1)
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

void ModelDjDq::deleteInstance()
{
  deleteObject(&modelDjDq);
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
  world_H_rootLink = modelState->getRootWorldRotoTranslation();
  refLink = mxArrayToString(prhs[1]);
  int refLinkID;
  std::string com("com");

  if(com.compare(refLink)==0)
  {
    refLinkID = -1;
  }
  else
  {
    robotModel->getFrameList().idToIndex(refLink,refLinkID);
  }

  if(!robotModel->computeDJdq(qj,world_H_rootLink,qjDot,vb,refLinkID,Djdq))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI DJDq call");
  }

  return(true);
}

bool ModelDjDq::processArguments(int nrhs, const mxArray* prhs[])
{

  size_t numDof = modelState->dof();

  if(mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1  || mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != numDof || mxGetN(prhs[4]) != 1 || mxGetM(prhs[5]) != 6 || mxGetN(prhs[5]) != 1 || !(mxIsChar(prhs[6])))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions / inputs");
  }
  robotModel = modelState->robotModel();
  qj = mxGetPr(prhs[3]);
  qjDot = mxGetPr(prhs[4]);
  vb = mxGetPr(prhs[5]);
  refLink = mxArrayToString(prhs[6]);

     double *R_temp,*p_temp;
  R_temp = (double *)mxGetPr(prhs[1]);
  p_temp = (double *)mxGetPr(prhs[2]);

  double tempR[9],tempP[3];
  
  for(int i = 0;i<3;i++)
  {
     tempP[i] = p_temp[i];
  }
  
  reorderMatrixElements(R_temp, tempR);
  
  wbi::Rotation tempRot(tempR);
  wbi::Frame tempFrame(tempRot, tempP);

#ifdef DEBUG
  mexPrintf("qj received \n");

  for(size_t i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif

  world_H_rootLink = tempFrame;

  if(Djdq != NULL)
  {
    int refLinkID;
    std::string com("com");

    if(com.compare(refLink)==0)
    {
      refLinkID = -1;
    }
    else
    {
      robotModel->getFrameList().idToIndex(refLink,refLinkID);
    }
    if(!robotModel->computeDJdq(qj,world_H_rootLink,qjDot,vb,refLinkID,Djdq))
    {
       mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI DJDq call");
    }
  }

  return true;
}

