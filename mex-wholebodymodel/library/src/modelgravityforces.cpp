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
#include <wbi/iWholeBodyModel.h>
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>

//local includes
#include "modelgravityforces.h"

using namespace mexWBIComponent;

ModelGravityForces *ModelGravityForces::modelGravityForces;
double ModelGravityForces::vb_0[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

ModelGravityForces::ModelGravityForces() : ModelComponent(3,0,1)
{
#ifdef DEBUG
  mexPrintf("ModelGravityForces constructed\n");
#endif
  g = modelState->g();
}

ModelGravityForces::~ModelGravityForces()
{
}

ModelGravityForces* ModelGravityForces::getInstance()
{
  if(modelGravityForces == NULL)
  {
    modelGravityForces = new ModelGravityForces;
  }
  return modelGravityForces;
}

void ModelGravityForces::deleteInstance()
{
  deleteObject(&modelGravityForces);
}

bool ModelGravityForces::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGravityForces\n");
#endif
  int numDof = modelState->dof();

  plhs[0] = mxCreateNumericMatrix(numDof+6, 1, mxDOUBLE_CLASS, mxREAL);
  h = mxGetPr(plhs[0]);

  return true;
}

bool ModelGravityForces::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to compute gravity forces\n");
#endif
  processArguments(nrhs, prhs);
  return true;
}

bool ModelGravityForces::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to fast compute gravity forces\n");
#endif
  robotModel = modelState->robotModel();

  qj = modelState->qj();

  size_t numDof   = modelState->dof();
  double *qjDot_0 = new double[numDof];
  memset(qjDot_0, 0, numDof*sizeof(double));

  g = modelState->g();
  world_H_rootLink = modelState->getRootWorldRotoTranslation();

  if( !robotModel->computeGeneralizedBiasForces(qj, world_H_rootLink, qjDot_0, vb_0, g, h) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call");
  }

  delete[] qjDot_0;
  qjDot_0 = NULL;

  return true;
}

bool ModelGravityForces::processArguments(int nrhs, const mxArray **prhs)
{
  size_t numDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
      mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions in ModelGeneralisedForces call");
  }
  double *pR, *ppos;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);

  double R_ro[9];
  reorderMatrixElements(pR, R_ro);

  wbi::Rotation rotm(R_ro);
  wbi::Frame tform(rotm, ppos);
  robotModel = modelState->robotModel();

  qj = mxGetPr(prhs[3]);

  double *qjDot_0 = new double[numDof];
  memset(qjDot_0, 0, numDof*sizeof(double));

#ifdef DEBUG
  mexPrintf("qj received\n");

  for(size_t i = 0; i < numDof; i++)
  {
    mexPrintf(" %f", *(qj + i));
  }
#endif

  world_H_rootLink = tform;
  g = modelState->g();

  if(h != NULL)
  {
    if( !robotModel->computeGeneralizedBiasForces(qj, world_H_rootLink, qjDot_0, vb_0, g, h) )
    {
      mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call");
    }
  }

  delete[] qjDot_0;
  qjDot_0 = NULL;

  return true;
}
