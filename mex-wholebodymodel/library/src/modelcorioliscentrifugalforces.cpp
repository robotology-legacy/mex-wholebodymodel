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
#include "modelcorioliscentrifugalforces.h"

using namespace mexWBIComponent;

ModelCoriolisCentrifugalForces *ModelCoriolisCentrifugalForces::modelCoriolisCentrifugalForces;
double ModelCoriolisCentrifugalForces::g_0[3] = {0.0f, 0.0f, 0.0f};

ModelCoriolisCentrifugalForces::ModelCoriolisCentrifugalForces() : ModelComponent(5, 0, 1)
{
#ifdef DEBUG
  mexPrintf("ModelCoriolisCentrifugalForces constructed\n");
#endif
}

ModelCoriolisCentrifugalForces::~ModelCoriolisCentrifugalForces()
{
}

ModelCoriolisCentrifugalForces *ModelCoriolisCentrifugalForces::getInstance()
{
  if(modelCoriolisCentrifugalForces == NULL)
    modelCoriolisCentrifugalForces = new ModelCoriolisCentrifugalForces;

  return modelCoriolisCentrifugalForces;
}

void ModelCoriolisCentrifugalForces::deleteInstance()
{
  deleteObject(&modelCoriolisCentrifugalForces);
}

bool ModelCoriolisCentrifugalForces::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelCoriolisCentrifugalForces\n");
#endif
  int numDof = modelState->dof();

  plhs[0] = mxCreateDoubleMatrix(numDof+6, 1, mxREAL);
  h = mxGetPr(plhs[0]);

  return true;
}

bool ModelCoriolisCentrifugalForces::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to compute Coriolis with centrifugal forces\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelCoriolisCentrifugalForces::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to fast compute Coriolis with centrifugal forces\n");
#endif
#ifdef DEBUG
  if(h == NULL) return false;
#endif
  robotModel = modelState->robotModel();

  qj    = modelState->qj();
  qjDot = modelState->qjDot();

  world_H_rootLink = modelState->getRootWorldRotoTranslation();
  vb = modelState->vb();

  if( !robotModel->computeGeneralizedBiasForces(qj, world_H_rootLink, qjDot, vb, g_0, h) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call");

  return true;
}

bool ModelCoriolisCentrifugalForces::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if(h == NULL) return false;
#endif
  size_t numDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
      mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != numDof || mxGetN(prhs[4]) != 1 ||
      mxGetM(prhs[5]) != 6 || mxGetN(prhs[5]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions in ModelCoriolisCentrifugalForces call");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);

  qj    = mxGetPr(prhs[3]);
  qjDot = mxGetPr(prhs[4]);
  vb    = mxGetPr(prhs[5]);

#ifdef DEBUG
  mexPrintf("qj received\n");

  for(size_t i=0; i < numDof; i++)
    mexPrintf(" %f", *(qj + i));
#endif

  double R_ro[9];
  reorderMatrixInRowMajor(pR, R_ro);

  wbi::Rotation rotm(R_ro);
  world_H_rootLink = wbi::Frame(rotm, ppos);

  if( !robotModel->computeGeneralizedBiasForces(qj, world_H_rootLink, qjDot, vb, g_0, h) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call");

  return true;
}
