/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
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
#include <wbi/iWholeBodyModel.h>
#include <mex.h>


// local includes
#include "modelupdatestate.h"
#include "modelstate.h"


using namespace mexWBIComponent;

ModelUpdateState *ModelUpdateState::modelUpdateState;

ModelUpdateState::ModelUpdateState() : ModelComponent(3,3,0)
{
#ifdef DEBUG
  mexPrintf("ModelUpdateState constructed \n");
#endif

}

ModelUpdateState * ModelUpdateState::getInstance()
{
  if(modelUpdateState == NULL)
  {
    modelUpdateState = new ModelUpdateState;
  }

  return(modelUpdateState);
}

void ModelUpdateState::deleteInstance()
{
  deleteObject(&modelUpdateState);
}

ModelUpdateState::~ModelUpdateState()
{

}

bool ModelUpdateState::compute(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelUpdateState\n");
#endif
  robotModel = modelState->robotModel();
  bool retVal = setState(nrhs,prhs);
  return(retVal);

}

bool ModelUpdateState::computeFast(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelUpdateState\n");
#endif
  robotModel = modelState->robotModel();
  bool retVal = setState(nrhs,prhs);
  return(retVal);
}

bool ModelUpdateState::allocateReturnSpace(int nlhs, mxArray* plhs[])
{

#ifdef DEBUG
  mexPrintf("Trying to allocate memory in ModelUpdateState (nothing to do really)\n");
#endif
  return(true);

}

bool ModelUpdateState::setState(int nrhs, const mxArray* prhs[])
{
    size_t numDof = modelState->dof();

    if(mxGetM(prhs[1])!=numDof || mxGetN(prhs[1])!=1 || mxGetM(prhs[2])!=numDof ||mxGetN(prhs[2])!=1 || mxGetM(prhs[3])!=6 || mxGetN(prhs[3])!=1)
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions / inputs");
    }
#ifdef DEBUG
       mexPrintf("Updating state\n");
#endif
     robotModel = modelState->robotModel();
     double *q_temp,*dq_temp,*dxb_temp;
      q_temp = (double *)mxGetPr(prhs[1]);
      dq_temp = (double *)mxGetPr(prhs[2]);
      dxb_temp = (double *)mxGetPr(prhs[3]);

      modelState->setState(q_temp,dq_temp,dxb_temp);

#ifdef DEBUG
      mexPrintf("Updated state : (q,dq)\n");

      mexPrintf("Updated State\n");
#endif
  return(true);
}
