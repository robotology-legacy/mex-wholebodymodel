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

// global includes

// library includes
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>

// local includes
#include "modeljacobian.h"

using namespace mexWBIComponent;

ModelJacobian *ModelJacobian::modelJacobian;

ModelJacobian::ModelJacobian() : ModelComponent(4, 1, 1)
{
#ifdef DEBUG
  mexPrintf("ModelJacobian constructed\n");
#endif
  j_rowMajor = NULL;
}

ModelJacobian::~ModelJacobian()
{
  if(j_rowMajor != NULL) {
    delete[] j_rowMajor;
    j_rowMajor = NULL;
  }
}

bool ModelJacobian::allocateReturnSpace(int nlhs, mxArray *plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelJacobian\n");
#endif
  int nCols = modelState->dof() + 6;

  plhs[0]    = mxCreateDoubleMatrix(6, nCols, mxREAL);
  j_colMajor = mxGetPr(plhs[0]);

  if(j_rowMajor == NULL)
    j_rowMajor = new double[6*nCols];

  return true;
}

ModelJacobian *ModelJacobian::getInstance()
{
  if(modelJacobian == NULL)
    modelJacobian = new ModelJacobian;

  return modelJacobian;
}

void ModelJacobian::deleteInstance()
{
  deleteObject(&modelJacobian);
}

bool ModelJacobian::compute(int nrhs, const mxArray * prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelJacobian\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelJacobian::computeFast(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to fast compute ModelJacobian\n");
#endif
#ifdef DEBUG
  if( (j_rowMajor == NULL) || (j_colMajor == NULL) ) return false;
#endif
  if( !mxIsChar(prhs[1]) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components");

  int numDof = modelState->dof();

  qj = modelState->qj();
  world_H_rootLink = modelState->getRootWorldRotoTranslation();

  refLink = mxArrayToString(prhs[1]);
  robotModel = modelState->robotModel();

  std::string com("com");
  int refLinkID = -1; // if refLink = "com"

  if(com.compare(refLink) != 0)
    robotModel->getFrameList().idToIndex(refLink, refLinkID);

  if( !(robotModel->computeJacobian(qj, world_H_rootLink, refLinkID, j_rowMajor)) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the jacobian call");

  // since Matlab uses the column-major order for multi-dimensional arrays,
  // we have to make an array-transposition ...
  reorderMatrixInColMajor(j_rowMajor, j_colMajor, 6, numDof+6);

#ifdef DEBUG
  mexPrintf("ModelJacobian fast computed\n");
#endif
  return true;
}

bool ModelJacobian::processArguments(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  if( (j_rowMajor == NULL) || (j_colMajor == NULL) ) return false;
#endif
  size_t numDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
      mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || !mxIsChar(prhs[4]))
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components");
  }
  robotModel = modelState->robotModel();

  double *R_temp, *p_temp;
  R_temp = mxGetPr(prhs[1]);
  p_temp = mxGetPr(prhs[2]);

  qj      = mxGetPr(prhs[3]);
  refLink = mxArrayToString(prhs[4]);

#ifdef DEBUG
  mexPrintf("qj received\n");

  for (size_t i=0; i < numDof; i++)
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

  if( !(robotModel->computeJacobian(qj, world_H_rootLink, refLinkID, j_rowMajor)) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the jacobian call");

  reorderMatrixInColMajor(j_rowMajor, j_colMajor, 6, numDof+6);
  return true;
}
