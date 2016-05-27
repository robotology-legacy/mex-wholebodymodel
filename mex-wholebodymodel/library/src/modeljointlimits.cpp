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

#include "../include/modeljointlimits.h"

#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <stdio.h>
#include <mex.h>

using namespace mexWBIComponent;
ModelJointLimits* ModelJointLimits::modelJointLimits = 0;


ModelJointLimits::ModelJointLimits() : ModelComponent(0,0,2)
{
#ifdef DEBUG
  mexPrintf("ModelJointLimits constructed \n");
#endif

}

bool ModelJointLimits::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelJointLimits\n");
#endif
  int numDof = modelState->dof();

  bool returnVal = false;
  if(nlhs!=2)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","2 output arguments required for joint limits");
  }

  plhs[0]=mxCreateDoubleMatrix(numDof,1, mxREAL);
  plhs[1]=mxCreateDoubleMatrix(numDof,1, mxREAL);
  returnVal = true;

  jointLowerLimit = mxGetPr(plhs[0]);
  jointUpperLimit = mxGetPr(plhs[1]);
  returnVal = true;
  return(returnVal);
}

ModelJointLimits::~ModelJointLimits()
{
#ifdef DEBUG
  mexPrintf("ModelJointLimits destructed\n");
#endif
}

ModelJointLimits* ModelJointLimits::getInstance()
{
  if(modelJointLimits == NULL)
  {
    modelJointLimits  = new ModelJointLimits;
  }
  return(modelJointLimits);
}

void ModelJointLimits::deleteInstance()
{
  deleteObject(&modelJointLimits);
}


bool ModelJointLimits::compute(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
   mexPrintf("Trying to compute ModelJointLimits\n");
#endif
  robotModel = modelState->robotModel();
  robotModel->getJointLimits(jointLowerLimit,jointUpperLimit);

  return(true);
}

bool ModelJointLimits::computeFast(int, const mxArray*[])
{
  robotModel = modelState->robotModel();
  robotModel->getJointLimits(jointLowerLimit,jointUpperLimit);

  return(true);
}
