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
//#include <cstring>

// library includes

// local includes
#include "modelgravitybiasforces.h"

using namespace mexWBIComponent;

ModelGravityBiasForces *ModelGravityBiasForces::modelGravityBiasForces = 0;

double *ModelGravityBiasForces::qj  = 0;
double *ModelGravityBiasForces::g   = 0;
double *ModelGravityBiasForces::g_q = 0;

ModelGravityBiasForces::ModelGravityBiasForces() : ModelComponent(3, 0, 1)
{
#ifdef DEBUG
  mexPrintf("ModelGravityBiasForces constructed.\n");
#endif
}

ModelGravityBiasForces::~ModelGravityBiasForces()
{
}

ModelGravityBiasForces *ModelGravityBiasForces::getInstance()
{
  if (modelGravityBiasForces == 0) {
    modelGravityBiasForces = new ModelGravityBiasForces;
  }
  return modelGravityBiasForces;
}

void ModelGravityBiasForces::deleteInstance()
{
  deleteObject(&modelGravityBiasForces);
}

bool ModelGravityBiasForces::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGravityBiasForces.\n");
#endif
  int nDof = modelState->dof();

  plhs[0] = mxCreateDoubleMatrix(nDof+6, 1, mxREAL);
  g_q = mxGetPr(plhs[0]);

  return true;
}

bool ModelGravityBiasForces::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGravityBiasForces performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelGravityBiasForces::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGravityBiasForces performing computeFast.\n");
#endif
#ifdef DEBUG
  if (g_q == 0) {
    return false;
  }
#endif
  robotModel = modelState->robotModel();

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();
  g      = modelState->g();

  if ( !robotModel->computeGravityBiasForces(qj, wf_H_b, g, g_q) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGravityBiasForces call.");
  }
  return true;
}

bool ModelGravityBiasForces::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (g_q == 0) {
    return false;
  }
#endif
  size_t nDof = modelState->dof();

  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
       mxGetM(prhs[3]) != nDof || mxGetN(prhs[3]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions in ModelGravityBiasForces call.");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);
  qj   = mxGetPr(prhs[3]);
  g    = modelState->g();

#ifdef DEBUG
  mexPrintf("qj received.\n");

  for (size_t i=0; i < nDof; i++) {
    mexPrintf(" %f", *(qj + i));
  }
#endif

  double R_rmo[9];
  reorderMatrixInRowMajor(pR, R_rmo); // matrix in "row major order"
  wbi::Rotation rot3d(R_rmo);

  wf_H_b = wbi::Frame(rot3d, ppos);

  if ( !robotModel->computeGravityBiasForces(qj, wf_H_b, g, g_q) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGravityBiasForces call.");
  }
  return true;
}
