/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: June, 2016
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

// library includes
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>

// local includes
#include "modeldjdq.h"

using namespace mexWBIComponent;

ModelDjDq *ModelDjDq::modelDjDq;

ModelDjDq::ModelDjDq(void) : ModelComponent(6, 1, 1)
{
#ifdef DEBUG
  mexPrintf("ModelGeneralisedBiasForces constructed\n");
#endif
}

ModelDjDq::~ModelDjDq()
{
}

ModelDjDq *ModelDjDq::getInstance(void)
{
  if(modelDjDq == NULL)
    modelDjDq = new ModelDjDq;

  return modelDjDq;
}

void ModelDjDq::deleteInstance()
{
  deleteObject(&modelDjDq);
}

bool ModelDjDq::allocateReturnSpace(int nlhs, mxArray *plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelDjDq\n");
#endif
  if(nlhs != 1)
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "one output argument required for computing ModelDjDq.");

  plhs[0] = mxCreateNumericMatrix(6, 1, mxDOUBLE_CLASS, mxREAL);
  Djdq = mxGetPr(plhs[0]);

  return true;
}

bool ModelDjDq::compute(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelDjDq\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelDjDq::computeFast(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelDjDq\n");
#endif
#ifdef DEBUG
  if(Djdq == NULL) return false;
#endif
  if( !mxIsChar(prhs[1]) )
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components");

  robotModel = modelState->robotModel();

  qj    = modelState->qj();
  qjDot = modelState->qjDot();
  vb    = modelState->vb();

  world_H_rootLink = modelState->getRootWorldRotoTranslation();

  refLink = mxArrayToString(prhs[1]);
  std::string com("com");
  int refLinkID = -1; // if refLink = "com"

  if(com.compare(refLink) != 0)
  {
    // get the index number ...
    robotModel->getFrameList().idToIndex(refLink, refLinkID);
  }

  if( !robotModel->computeDJdq(qj, world_H_rootLink, qjDot, vb, refLinkID, Djdq) )
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs", "Something failed in the WBI DJDq call");

  return true;
}

bool ModelDjDq::processArguments(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  if(Djdq == NULL) return false;
#endif
  size_t numDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
      mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != numDof || mxGetN(prhs[4]) != 1 ||
      mxGetM(prhs[5]) != 6 || mxGetN(prhs[5]) != 1 || !(mxIsChar(prhs[6])) )
  {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions / inputs");
  }
  robotModel = modelState->robotModel();

  double *R_temp, *p_temp;
  R_temp = mxGetPr(prhs[1]);
  p_temp = mxGetPr(prhs[2]);

  qj      = mxGetPr(prhs[3]);
  qjDot   = mxGetPr(prhs[4]);
  vb      = mxGetPr(prhs[5]);
  refLink = mxArrayToString(prhs[6]);

#ifdef DEBUG
  mexPrintf("qj received \n");

  for(size_t i=0; i < numDof; i++)
    mexPrintf(" %f", *(qj + i));
#endif

  double tempR[9];
  reorderMatrixInRowMajor(R_temp, tempR);
  wbi::Rotation tempRot(tempR);

  world_H_rootLink = wbi::Frame(tempRot, p_temp);

  std::string com("com");
  int refLinkID = -1; // if refLink = "com"

  if(com.compare(refLink) != 0)
    robotModel->getFrameList().idToIndex(refLink, refLinkID);

  if( !robotModel->computeDJdq(qj, world_H_rootLink, qjDot, vb, refLinkID, Djdq) )
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs", "Something failed in the WBI DJDq call");

  return true;
}
