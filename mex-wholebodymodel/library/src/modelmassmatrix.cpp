/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * E-mail: naveen.kuppuswamy@iit.it
 *
 * Modified by: Martin Neururer
 * E-mail:      martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
 * Date:        June, 2016 & January, 2017
 *
 * The development of this software was supported by the FP7 EU-project
 * CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and Robotics (b)),
 * <http://www.codyco.eu>.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * A copy of the GNU General Public License can be found along with
 * the source library. If not, see <http://www.gnu.org/licenses/>.
 */

// global includes

// library includes

// local includes
#include "modelmassmatrix.h"

using namespace mexWBIComponent;

ModelMassMatrix *ModelMassMatrix::modelMassMatrix = 0;

double *ModelMassMatrix::qj = 0;
double *ModelMassMatrix::M  = 0;

ModelMassMatrix::ModelMassMatrix() : ModelComponent(3, 0, 1)
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix constructed.\n");
#endif
}

ModelMassMatrix::~ModelMassMatrix()
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix destructed.\n");
#endif
}

bool ModelMassMatrix::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelMassMatrix.\n");
#endif
  int dim = modelState->dof() + 6;

  plhs[0] = mxCreateDoubleMatrix(dim, dim, mxREAL);
  M = mxGetPr(plhs[0]);

  return true;
}

ModelMassMatrix *ModelMassMatrix::getInstance()
{
  if (modelMassMatrix == 0) {
    modelMassMatrix = new ModelMassMatrix;
  }
  return modelMassMatrix;
}

void ModelMassMatrix::deleteInstance()
{
  deleteObject(&modelMassMatrix);
}

bool ModelMassMatrix::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelMassMatrix::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix performing computeFast.\n");
#endif
#ifdef DEBUG
  if (M == 0) {
    return false;
  }
#endif
  robotModel = modelState->robotModel();

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();

  if ( !robotModel->computeMassMatrix(qj, wf_H_b, M) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeMassMatrix call.");
  }

#ifdef DEBUG
  mexPrintf("ModelMassMatrix fast computed.\n");
#endif
  return true;
}

bool ModelMassMatrix::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (M == 0) {
    return false;
  }
#endif
  size_t nDof = modelState->dof();

  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 ||
       mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != nDof || mxGetN(prhs[3]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions.");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);
  qj   = mxGetPr(prhs[3]);

#ifdef DEBUG
  mexPrintf("qj received.\n");

  for (size_t i=0; i < nDof; i++) {
    mexPrintf(" %f", *(qj + i));
  }
#endif

  double tempR[9];
  reorderMatrixInRowMajor(pR, tempR);
  wbi::Rotation tempRot(tempR);

  wf_H_b = wbi::Frame(tempRot, ppos);

  if ( !robotModel->computeMassMatrix(qj, wf_H_b, M) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeMassMatrix call.");
  }
  return true;
}
