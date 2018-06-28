/*
 * Copyright (C) 2016 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Martin Neururer
 * E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
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
#include "modelinversedynamics.h"

using namespace mexWBIComponent;

ModelInverseDynamics *ModelInverseDynamics::modelInverseDynamics = 0;

double *ModelInverseDynamics::qj      = 0;
double *ModelInverseDynamics::qj_dot  = 0;
double *ModelInverseDynamics::qj_ddot = 0;
double *ModelInverseDynamics::vb      = 0;
double *ModelInverseDynamics::vb_dot  = 0;
double *ModelInverseDynamics::g       = 0;
double *ModelInverseDynamics::tau     = 0;

ModelInverseDynamics::ModelInverseDynamics() : ModelComponent(7, 2, 1)
{
#ifdef DEBUG
  mexPrintf("ModelInverseDynamics constructed.\n");
#endif
}

ModelInverseDynamics::~ModelInverseDynamics()
{
}

ModelInverseDynamics *ModelInverseDynamics::getInstance()
{
  if (modelInverseDynamics == 0) {
    modelInverseDynamics = new ModelInverseDynamics;
  }
  return modelInverseDynamics;
}

void ModelInverseDynamics::deleteInstance()
{
  deleteObject(&modelInverseDynamics);
}

bool ModelInverseDynamics::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelInverseDynamics.\n");
#endif
  int nDof = modelState->dof();

  plhs[0] = mxCreateDoubleMatrix(nDof+6, 1, mxREAL);
  tau = mxGetPr(plhs[0]);

  return true;
}

bool ModelInverseDynamics::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelInverseDynamics performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelInverseDynamics::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelInverseDynamics performing computeFast.\n");
#endif
#ifdef DEBUG
  if (tau == 0) {
    return false;
  }
#endif
  return processFastArguments(nrhs, prhs);
}

bool ModelInverseDynamics::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (tau == 0) {
    return false;
  }
#endif
  size_t nDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != nDof ||
      mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != nDof || mxGetN(prhs[4]) != 1 || mxGetM(prhs[5]) != 6 || mxGetN(prhs[5]) != 1 ||
      mxGetM(prhs[6]) != nDof || mxGetN(prhs[6]) != 1 || mxGetM(prhs[7]) != 6 || mxGetN(prhs[7]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions in ModelInverseDynamics call.");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR      = mxGetPr(prhs[1]);
  ppos    = mxGetPr(prhs[2]);
  qj      = mxGetPr(prhs[3]);
  qj_dot  = mxGetPr(prhs[4]);
  vb      = mxGetPr(prhs[5]);
  qj_ddot = mxGetPr(prhs[6]);
  vb_dot  = mxGetPr(prhs[7]);
  g       = modelState->g();

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

  if ( !robotModel->inverseDynamics(qj, wf_H_b, qj_dot, vb, qj_ddot, vb_dot, g, tau) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI inverseDynamics call.");
  }
  return true;
}

bool ModelInverseDynamics::processFastArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (tau == 0) {
    return false;
  }
#endif
  size_t nDof = modelState->dof();

  if( mxGetM(prhs[1]) != nDof || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 6 || mxGetN(prhs[2]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions in ModelInverseDynamics call.");
  }
  robotModel = modelState->robotModel();

  qj_ddot = mxGetPr(prhs[1]);
  vb_dot  = mxGetPr(prhs[2]);

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();
  qj_dot = modelState->qj_dot();
  vb     = modelState->vb();
  g      = modelState->g();

  if ( !robotModel->inverseDynamics(qj, wf_H_b, qj_dot, vb, qj_ddot, vb_dot, g, tau) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI inverseDynamics call.");
  }
  return true;
}
