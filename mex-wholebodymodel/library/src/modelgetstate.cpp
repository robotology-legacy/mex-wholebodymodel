/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: June, 2016 & January, 2017
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

// global includes

// library includes

// local includes
#include "modelgetstate.h"

using namespace mexWBIComponent;

ModelGetState *ModelGetState::modelGetState = 0;

double ModelGetState::quat_b[4] = {0.0f, 0.0f, 0.0f, 0.0f};
double *ModelGetState::vqT_b    = 0;
double *ModelGetState::qj       = 0;
double *ModelGetState::vb       = 0;
double *ModelGetState::qj_dot   = 0;

ModelGetState::ModelGetState() : ModelComponent(0, 1, 4)
{
#ifdef DEBUG
  mexPrintf("ModelGetState constructuted.\n");
#endif
}

ModelGetState::~ModelGetState()
{
}

ModelGetState *ModelGetState::getInstance()
{
  if (modelGetState == 0) {
    modelGetState = new ModelGetState;
  }
  return modelGetState;
}

void ModelGetState::deleteInstance()
{
  deleteObject(&modelGetState);
}

bool ModelGetState::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGetState.\n");
#endif
  if (nlhs != 4) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumOutputs", "4 output arguments required for ModelGetState.");
  }
  int numDof = modelState->dof();

  plhs[0] = mxCreateDoubleMatrix(7, 1, mxREAL);      // vqT_b
  plhs[1] = mxCreateDoubleMatrix(numDof, 1, mxREAL); // qj
  plhs[2] = mxCreateDoubleMatrix(6, 1, mxREAL);      // vb
  plhs[3] = mxCreateDoubleMatrix(numDof, 1, mxREAL); // qj_dot

  vqT_b  = mxGetPr(plhs[0]);
  qj     = mxGetPr(plhs[1]);
  vb     = mxGetPr(plhs[2]);
  qj_dot = mxGetPr(plhs[3]);

  return true;
}

bool ModelGetState::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGetState performing compute.\n");
#endif
  wf_H_b = modelState->getBase2WorldTransformation();

#ifdef DEBUG
  mexPrintf("Inside getState - RootWorldRotoTrans:\n");
  mexPrintf("wf_H_b\n");
  mexPrintf((wf_H_b.R.toString()).c_str());
  mexPrintf(" = \n");
  mexPrintf("wf_H_lnk\n");
  mexPrintf(( (modelState->getReferenceToWorldFrameRotoTrans()).R.toString()).c_str());
  mexPrintf("\n\n");
#endif

  wf_H_b.R.getQuaternion(quat_b[1], quat_b[2], quat_b[3], quat_b[0]);

#ifdef DEBUG
  std::stringstream ssR;
  ssR << "quaternion: [" << quat_b[0] << "," << quat_b[1] << "," << quat_b[2] << "," << quat_b[3] << "]\n";
  std::string sR = ssR.str();
  mexPrintf(sR.c_str());
#endif

  int i;
  for (i=0; i < 3; i++) {
    *(vqT_b + i) = *(wf_H_b.p + i);
  }

  for (i=0; i < 4; i++) {
    *(vqT_b + (3+i)) = *(quat_b + i);
  }

  modelState->qj(qj);
  modelState->vb(vb);
  modelState->qj_dot(qj_dot);

  return true;
}

bool ModelGetState::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGetState performing computeFast.\n");
#endif
  wf_H_b = modelState->getBase2WorldTransformation();

#ifdef DEBUG
  mexPrintf("Inside of getState - RootWorldRotoTrans:\n");
  mexPrintf("wf_H_b\n");
  mexPrintf((wf_H_b.R.toString()).c_str());
  mexPrintf(" = \n");
  mexPrintf("wf_H_lnk\n");
  mexPrintf(( (modelState->getReferenceToWorldFrameRotoTrans()).R.toString()).c_str());
  mexPrintf("\n\n");
#endif

  (modelState->getBase2WorldTransformation()).R.getQuaternion(quat_b[1], quat_b[2], quat_b[3], quat_b[0]);

#ifdef DEBUG
  std::stringstream ssR;
  ssR << "quaternion: [" << quat_b[0] << "," << quat_b[1] << "," << quat_b[2] << "," << quat_b[3] << "]\n";
  std::string sR = ssR.str();
  mexPrintf(sR.c_str());
#endif

  int i;
  for (i=0; i < 3; i++) {
    *(vqT_b + i) = *(wf_H_b.p + i);
  }

  for (i=0; i < 4; i++) {
    *(vqT_b + (3+i)) = *(quat_b + i);
  }

  modelState->qj(qj);
  modelState->vb(vb);
  modelState->qj_dot(qj_dot);

  return true;
}
