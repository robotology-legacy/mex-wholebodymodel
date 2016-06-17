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

// local includes
#include "modelmassmatrix.h"

using namespace mexWBIComponent;

ModelMassMatrix *ModelMassMatrix::modelMassMatrix = 0;

ModelMassMatrix::ModelMassMatrix() : ModelComponent(3, 0, 1)
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix constructed\n");
#endif
}

ModelMassMatrix::~ModelMassMatrix()
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix destructed\n");
#endif
}

bool ModelMassMatrix::allocateReturnSpace(int nlhs, mxArray *plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelMassMatrix\n");
#endif
  int dim = modelState->dof() + 6;

  plhs[0]    = mxCreateDoubleMatrix(dim, dim, mxREAL);
  massMatrix = mxGetPr(plhs[0]);

  return true;
}

ModelMassMatrix *ModelMassMatrix::getInstance()
{
  if (modelMassMatrix == NULL)
    modelMassMatrix = new ModelMassMatrix;

  return modelMassMatrix;
}

void ModelMassMatrix::deleteInstance()
{
  deleteObject(&modelMassMatrix);
}

bool ModelMassMatrix::compute(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelMassMatrix\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelMassMatrix::computeFast(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to fast compute ModelMassMatrix\n");
#endif
#ifdef DEBUG
  if(massMatrix == NULL) return false;
#endif
  robotModel = modelState->robotModel();
  qj = modelState->qj();
  world_H_rootLink = modelState->getRootWorldRotoTranslation();

  if( !robotModel->computeMassMatrix(qj, world_H_rootLink, massMatrix) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI MassMatrix call");

#ifdef DEBUG
  mexPrintf("ModelMassMatrix fast computed\n");
#endif

  return true;
}

bool ModelMassMatrix::processArguments(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  if(massMatrix == NULL) return false;
#endif
  size_t numDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 ||
      mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions");
  }
  robotModel = modelState->robotModel();

  double *R_temp, *p_temp;
  R_temp = mxGetPr(prhs[1]);
  p_temp = mxGetPr(prhs[2]);

  qj = mxGetPr(prhs[3]);

#ifdef DEBUG
  mexPrintf("qj received\n");

  for (size_t i=0; i < numDof; i++)
    mexPrintf(" %f", *(qj + i));
#endif

  double tempR[9];
  reorderMatrixInRowMajor(R_temp, tempR);
  wbi::Rotation tempRot(tempR);

  world_H_rootLink = wbi::Frame(tempRot, p_temp);

  if( !robotModel->computeMassMatrix(qj, world_H_rootLink, massMatrix) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI MassMatrix call");

  return true;
}
