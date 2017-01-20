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

//double ModelGravityBiasForces::vb_0[6]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//double *ModelGravityBiasForces::qj_dot_0 = 0;
double *ModelGravityBiasForces::qj  = 0;
double *ModelGravityBiasForces::g   = 0;
double *ModelGravityBiasForces::g_v = 0;

ModelGravityBiasForces::ModelGravityBiasForces() : ModelComponent(3, 0, 1)
{
#ifdef DEBUG
  mexPrintf("ModelGravityBiasForces constructed.\n");
#endif
}

ModelGravityBiasForces::~ModelGravityBiasForces()
{
  // if (qj_dot_0 != 0) {
  //   delete[] qj_dot_0;
  //   qj_dot_0 = 0;
  // }
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
  int numDof = modelState->dof();

  plhs[0] = mxCreateDoubleMatrix(numDof+6, 1, mxREAL);
  g_v = mxGetPr(plhs[0]);

  // if (qj_dot_0 == 0) {
  //   qj_dot_0 = new double[numDof];
  //   memset(qj_dot_0, 0, numDof*sizeof(double));
  // }

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
  if (g_v == 0) {
    return false;
  }
#endif
  robotModel = modelState->robotModel();

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();
  g      = modelState->g();

  // if ( !robotModel->computeGeneralizedBiasForces(qj, wf_H_b, qj_dot_0, vb_0, g, g_v) )
  // {
  //   mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call.");
  // }
  if ( !robotModel->computeGravityBiasForces(qj, wf_H_b, g, g_v) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGravityBiasForces call.");
  }
  return true;
}

bool ModelGravityBiasForces::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (g_v == 0) {
    return false;
  }
#endif
  size_t numDof = modelState->dof();

  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
       mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 )
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

  for (size_t i=0; i < numDof; i++) {
    mexPrintf(" %f", *(qj + i));
  }
#endif

  double R_rmo[9];
  reorderMatrixInRowMajor(pR, R_rmo); // matrix in "row major order"
  wbi::Rotation rot3d(R_rmo);

  wf_H_b = wbi::Frame(rot3d, ppos);

  // if ( !robotModel->computeGeneralizedBiasForces(qj, wf_H_b, qj_dot_0, vb_0, g, g_v) ) {
  //   mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call.");
  // }
  if ( !robotModel->computeGravityBiasForces(qj, wf_H_b, g, g_v) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGravityBiasForces call.");
  }
  return true;
}
