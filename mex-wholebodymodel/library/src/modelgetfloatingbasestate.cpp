/*
 * Copyright (C) 2016 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Martin Neururer
 * email: martin.neururer@gmail.com, gabriele.nava@iit.it
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
// #include <wbi/iWholeBodyModel.h>
// #include <boost/concept_check.hpp>

//local includes
#include "modelgetfloatingbasestate.h"

using namespace mexWBIComponent;

ModelGetFloatingBaseState *ModelGetFloatingBaseState::modelGetFloatingBaseState;

ModelGetFloatingBaseState::ModelGetFloatingBaseState() : ModelComponent(0, 1, 3)
{
#ifdef DEBUG
  mexPrintf("ModelGetFloatingBaseState constructuted\n");
#endif
}

ModelGetFloatingBaseState::~ModelGetFloatingBaseState()
{
}

ModelGetFloatingBaseState *ModelGetFloatingBaseState::getInstance()
{
  if(modelGetFloatingBaseState == NULL)
    modelGetFloatingBaseState = new ModelGetFloatingBaseState;

  return modelGetFloatingBaseState;
}

void ModelGetFloatingBaseState::deleteInstance()
{
  deleteObject(&modelGetFloatingBaseState);
}

bool ModelGetFloatingBaseState::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGetFloatingBaseState\n");
#endif
  if(nlhs != 3)
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumOutputs", "3 output arguments required for ModelGetFloatingBaseState");

  plhs[0] = mxCreateDoubleMatrix(3,3, mxREAL);
  plhs[1] = mxCreateDoubleMatrix(3,1, mxREAL);
  plhs[2] = mxCreateDoubleMatrix(6,1, mxREAL);

  w_R_b = mxGetPr(plhs[0]);
  w_p_b = mxGetPr(plhs[1]);
  vb    = mxGetPr(plhs[2]);

  return true;
}

bool ModelGetFloatingBaseState::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGetFloatingBaseState performing Compute");
#endif
  world_H_rootLink = modelState->getRootWorldRotoTranslation();

#ifdef DEBUG
  mexPrintf("Inside getState\nRootWorldRotoTrans\n");
  mexPrintf("world_H_root\n");

  mexPrintf( (world_H_rootLink.R.toString()).c_str() );
  mexPrintf(" = \n");

  mexPrintf("world_H_ReferenceLink\n");
  mexPrintf( ((modelState->getReferenceToWorldFrameRotoTrans()).R.toString()).c_str() );
  mexPrintf("\n\n");
#endif

  double rotm[9]; // 3x3 rotation matrix array ...
  world_H_rootLink.R.getDcm(rotm);

#ifdef DEBUG
  std::stringstream ssR;
  ssR.setf(std::ios::floatfield, std::ios::fixed);
  ssR.precision(4);
  double dVal = 0.0f;

  ssR << "DCM :\n";
  for(int i=0; i < 9; i++)
  {
    dVal = *(rotm + i);
    if (dVal < 0)
      ssR << "  " << dVal;
    else
      ssR << "   " << dVal;

    if(((i+1) % 3) == 0) ssR << "\n";
  }

  std::string sR = ssR.str();
  mexPrintf(sR.c_str());
#endif

  // since the values in the array are stored in row-major order and Matlab
  // uses the column-major order for multi-dimensional arrays, we have to
  // make an array-transposition ...
  reorderMatrixInColMajor(rotm, w_R_b);

  memcpy(w_p_b, world_H_rootLink.p, 3*sizeof(double));
  modelState->vb(vb);

  return true;
}

bool ModelGetFloatingBaseState::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGetFloatingBaseState performing Compute Fast");
#endif
#ifdef DEBUG
  world_H_rootLink = modelState->getRootWorldRotoTranslation();

  mexPrintf("Inside getState\nRootWorldRotoTrans\n");
  mexPrintf("world_H_root\n");
  mexPrintf( (world_H_rootLink.R.toString()).c_str() );
  mexPrintf(" = \n");
  mexPrintf("world_H_ReferenceLink\n");
  mexPrintf( ((modelState->getReferenceToWorldFrameRotoTrans()).R.toString()).c_str() );
  mexPrintf("\n\n");
#endif
  double rotm[9];

  (modelState->getRootWorldRotoTranslation()).R.getDcm(rotm);
  reorderMatrixInColMajor(rotm, w_R_b);

  memcpy(w_p_b, (modelState->getRootWorldRotoTranslation()).p, 3*sizeof(double));
  modelState->vb(vb);

  return true;
}
