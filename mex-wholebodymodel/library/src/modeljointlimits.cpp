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
#include "modeljointlimits.h"

using namespace mexWBIComponent;

ModelJointLimits *ModelJointLimits::modelJointLimits = 0;

double *ModelJointLimits::jntlim_lower = 0;
double *ModelJointLimits::jntlim_upper = 0;

ModelJointLimits::ModelJointLimits() : ModelComponent(0, 0, 2)
{
#ifdef DEBUG
  mexPrintf("ModelJointLimits constructed.\n");
#endif
}

bool ModelJointLimits::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelJointLimits.\n");
#endif
  if (nlhs != 2) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "2 output arguments required for joint limits.");
  }
  int nDof = modelState->dof();

  plhs[0] = mxCreateDoubleMatrix(nDof, 1, mxREAL);
  plhs[1] = mxCreateDoubleMatrix(nDof, 1, mxREAL);

  jntlim_lower = mxGetPr(plhs[0]);
  jntlim_upper = mxGetPr(plhs[1]);

  return true;
}

ModelJointLimits::~ModelJointLimits()
{
#ifdef DEBUG
  mexPrintf("ModelJointLimits destructed.\n");
#endif
}

ModelJointLimits *ModelJointLimits::getInstance()
{
  if (modelJointLimits == 0) {
    modelJointLimits = new ModelJointLimits;
  }
  return modelJointLimits;
}

void ModelJointLimits::deleteInstance()
{
  deleteObject(&modelJointLimits);
}

bool ModelJointLimits::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelJointLimits performing compute.\n");
#endif
  return getJointLimits(jntlim_lower, jntlim_upper);
}

bool ModelJointLimits::computeFast(int nrhs, const mxArray **prhs)
{
  return getJointLimits(jntlim_lower, jntlim_upper);
}

bool ModelJointLimits::getJointLimits(double *pJntlim_lwr, double *pJntlim_upr)
{
  robotModel = modelState->robotModel();
  return robotModel->getJointLimits(pJntlim_lwr, pJntlim_upr);
}
