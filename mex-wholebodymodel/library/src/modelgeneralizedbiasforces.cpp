/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * E-mail: naveen.kuppuswamy@iit.it
 *
 * Modified by: Martin Neururer
 * E-mail:      martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
 * Date:        June, 2016 & January, 2017
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)),
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
#include "modelgeneralizedbiasforces.h"

using namespace mexWBIComponent;

ModelGeneralizedBiasForces *ModelGeneralizedBiasForces::modelGeneralizedBiasForces = 0;

double *ModelGeneralizedBiasForces::qj     = 0;
double *ModelGeneralizedBiasForces::qj_dot = 0;
double *ModelGeneralizedBiasForces::vb     = 0;
double *ModelGeneralizedBiasForces::g      = 0;
double *ModelGeneralizedBiasForces::c_qv   = 0;

ModelGeneralizedBiasForces::ModelGeneralizedBiasForces() : ModelComponent(5, 0, 1)
{
#ifdef DEBUG
  mexPrintf("ModelGeneralizedBiasForces constructed.\n");
#endif
}

ModelGeneralizedBiasForces::~ModelGeneralizedBiasForces()
{
}

ModelGeneralizedBiasForces *ModelGeneralizedBiasForces::getInstance()
{
  if (modelGeneralizedBiasForces == 0) {
    modelGeneralizedBiasForces = new ModelGeneralizedBiasForces;
  }
  return modelGeneralizedBiasForces;
}

void ModelGeneralizedBiasForces::deleteInstance()
{
  deleteObject(&modelGeneralizedBiasForces);
}

bool ModelGeneralizedBiasForces::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGeneralizedBiasForces.\n");
#endif
  int nDof = modelState->dof();

  plhs[0] = mxCreateNumericMatrix(nDof+6, 1, mxDOUBLE_CLASS, mxREAL);
  c_qv = mxGetPr(plhs[0]);

  return true;
}

bool ModelGeneralizedBiasForces::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGeneralizedBiasForces performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelGeneralizedBiasForces::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelGeneralizedBiasForces performing computeFast.\n");
#endif
#ifdef DEBUG
  if (c_qv == 0) {
    return false;
  }
#endif
  robotModel = modelState->robotModel();

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();
  qj_dot = modelState->qj_dot();
  vb     = modelState->vb();
  g      = modelState->g();

  if ( !robotModel->computeGeneralizedBiasForces(qj, wf_H_b, qj_dot, vb, g, c_qv) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call.");
  }
  return true;
}

bool ModelGeneralizedBiasForces::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (c_qv == 0) {
    return false;
  }
#endif
  size_t nDof = modelState->dof();

  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||  mxGetM(prhs[3]) != nDof ||
       mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != nDof || mxGetN(prhs[4]) != 1 || mxGetM(prhs[5]) != 6 || mxGetN(prhs[5]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions in ModelGeneralisedForces call.");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR     = mxGetPr(prhs[1]);
  ppos   = mxGetPr(prhs[2]);
  qj     = mxGetPr(prhs[3]);
  qj_dot = mxGetPr(prhs[4]);
  vb     = mxGetPr(prhs[5]);
  g      = modelState->g();

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

  if ( !robotModel->computeGeneralizedBiasForces(qj, wf_H_b, qj_dot, vb, g, c_qv) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeGeneralizedBiasForces call.");
  }
  return true;
}
