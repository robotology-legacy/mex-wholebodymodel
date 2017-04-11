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
#include "modelupdatestate.h"

using namespace mexWBIComponent;

ModelUpdateState *ModelUpdateState::modelUpdateState = 0;

ModelUpdateState::ModelUpdateState() : ModelComponent(3, 3, 0)
{
#ifdef DEBUG
  mexPrintf("ModelUpdateState constructed.\n");
#endif
}

ModelUpdateState *ModelUpdateState::getInstance()
{
  if (modelUpdateState == 0) {
    modelUpdateState = new ModelUpdateState;
  }
  return modelUpdateState;
}

void ModelUpdateState::deleteInstance()
{
  deleteObject(&modelUpdateState);
}

ModelUpdateState::~ModelUpdateState()
{
#ifdef DEBUG
  mexPrintf("ModelUpdateState destructed.\n");
#endif
}

bool ModelUpdateState::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelUpdateState performing compute.\n");
#endif
  robotModel = modelState->robotModel();
  return setState(nrhs, prhs);
}

bool ModelUpdateState::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelUpdateState performing computeFast.\n");
#endif
  robotModel = modelState->robotModel();
  return setState(nrhs, prhs);
}

bool ModelUpdateState::allocateReturnSpace(int nlhs, mxArray **plhs)
{
  // nothing to do really
  return true;
}

bool ModelUpdateState::setState(int nrhs, const mxArray **prhs)
{
  size_t nDof = modelState->dof();

  if ( mxGetM(prhs[1]) != nDof || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != nDof ||
       mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 6 || mxGetN(prhs[3]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/inputs.");
  }
  robotModel = modelState->robotModel();

  double *pqj, *pqj_dot, *pvb;
  pqj     = mxGetPr(prhs[1]);
  pqj_dot = mxGetPr(prhs[2]);
  pvb     = mxGetPr(prhs[3]);

  return modelState->setState(pqj, pqj_dot, pvb);
}
