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

// global includes

// library includes

// local includes
#include "modelinversedynamics.h"

using namespace mexWBIComponent;

ModelInverseDynamics *ModelInverseDynamics::modelInverseDynamics;

ModelInverseDynamics::ModelInverseDynamics() : ModelComponent(7, 0, 1)
{
#ifdef DEBUG
  mexPrintf("ModelInverseDynamics constructed\n");
#endif
  g = modelState->g();
}

ModelInverseDynamics::~ModelInverseDynamics()
{
}

ModelInverseDynamics *ModelInverseDynamics::getInstance()
{
  if(modelInverseDynamics == NULL)
    modelInverseDynamics = new ModelInverseDynamics;

  return modelInverseDynamics;
}

void ModelInverseDynamics::deleteInstance()
{
  deleteObject(&modelInverseDynamics);
}

bool ModelInverseDynamics::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelInverseDynamics\n");
#endif
  int numDof = modelState->dof();

  plhs[0] = mxCreateDoubleMatrix(numDof+6, 1, mxREAL);
  tau = mxGetPr(plhs[0]);

  return true;
}

bool ModelInverseDynamics::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to compute the inverse dynamics\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelInverseDynamics::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to fast compute the inverse dynamics\n");
#endif
#ifdef DEBUG
  if(tau == NULL) return false;
#endif
  size_t numDof = modelState->dof();
  robotModel    = modelState->robotModel();

  world_H_rootLink = modelState->getRootWorldRotoTranslation();
  qj    = modelState->qj();
  qjDot = modelState->qjDot();
  vb    = modelState->vb();

  qjDDot = new double[numDof];
  double vbDot[6];

  // calculate the joint accelerations and the acceleration (linear & angular)
  // of the robot base in world reference frame:
  qjDDot =
  vbDot  =

  g = modelState->g();

  if( !robotModel->inverseDynamics(qj, world_H_rootLink, qjDot, vb, qjDDot, vbDot, g, tau) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI inverseDynamics call");

  delete[] qjDDot;
  qjDDot = NULL;

  return true;
}

bool ModelInverseDynamics::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if(tau == NULL) return false;
#endif
  size_t numDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||mxGetM(prhs[3]) != numDof ||
      mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != numDof || mxGetN(prhs[4]) != 1 || mxGetM(prhs[5]) != 6 || mxGetN(prhs[5]) != 1 ||
      mxGetM(prhs[6]) != numDof || mxGetN(prhs[6]) != 1 || mxGetM(prhs[7]) != 6 || mxGetN(prhs[7]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions in ModelInverseDynamics call");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);

  qj     = mxGetPr(prhs[3]);
  qjDot  = mxGetPr(prhs[4]);
  vb     = mxGetPr(prhs[5]);
  qjDDot = mxGetPr(prhs[6]);
  vbDot  = mxGetPr(prhs[7]);

#ifdef DEBUG
  mexPrintf("qj received\n");

  for(size_t i=0; i < numDof; i++)
    mexPrintf(" %f", *(qj + i));
#endif

  double R_ro[9];
  reorderMatrixInRowMajor(pR, R_ro);

  wbi::Rotation rotm(R_ro);
  world_H_rootLink = wbi::Frame(rotm, ppos);

  g = modelState->g();

  if( !robotModel->inverseDynamics(qj, world_H_rootLink, qjDot, vb, qjDDot, vbDot, g, tau) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI inverseDynamics call");

  return true;
}
