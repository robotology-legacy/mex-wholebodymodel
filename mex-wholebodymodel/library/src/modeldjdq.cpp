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

//global includes

// library includes
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>

// local includes
#include "modeldjdq.h"

using namespace mexWBIComponent;

ModelDJdq *ModelDJdq::modelDJdq = 0;

double *ModelDJdq::qj     = 0;
double *ModelDJdq::qj_dot = 0;
double *ModelDJdq::vb     = 0;
char   *ModelDJdq::refLnk = 0;
double *ModelDJdq::dJdq   = 0;

ModelDJdq::ModelDJdq() : ModelComponent(6, 1, 1)
{
#ifdef DEBUG
  mexPrintf("ModelGeneralizedBiasForces constructed.\n");
#endif
}

ModelDJdq::~ModelDJdq()
{
}

ModelDJdq *ModelDJdq::getInstance()
{
  if (modelDJdq == 0) {
    modelDJdq = new ModelDJdq;
  }
  return modelDJdq;
}

void ModelDJdq::deleteInstance()
{
  deleteObject(&modelDJdq);
}

bool ModelDJdq::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelDJdq.\n");
#endif
  if (nlhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "one output argument required for computing ModelDJdq.");
  }
  plhs[0] = mxCreateNumericMatrix(6, 1, mxDOUBLE_CLASS, mxREAL);
  dJdq = mxGetPr(plhs[0]);

  return true;
}

bool ModelDJdq::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelDJdq performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelDJdq::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelDJdq performing computeFast.\n");
#endif
#ifdef DEBUG
  if (dJdq == 0) {
    return false;
  }
#endif
  if( !mxIsChar(prhs[1]) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components.");
  }

  robotModel = modelState->robotModel();

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();
  qj_dot = modelState->qj_dot();
  vb     = modelState->vb();
  refLnk = mxArrayToString(prhs[1]);

  std::string com("com");
  int refLnkID = -1; // if refLnk = "com"

  if (com.compare(refLnk) != 0) {
    // get the index number ...
    robotModel->getFrameList().idToIndex(refLnk, refLnkID);
  }

  if ( !robotModel->computeDJdq(qj, wf_H_b, qj_dot, vb, refLnkID, dJdq) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI DJDq call.");
  }
  return true;
}

bool ModelDJdq::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (dJdq == 0) {
    return false;
  }
#endif
  size_t numDof = modelState->dof();

  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != numDof ||
       mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != numDof || mxGetN(prhs[4]) != 1 || mxGetM(prhs[5]) != 6 ||
       mxGetN(prhs[5]) != 1 || !(mxIsChar(prhs[6])) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/inputs.");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR     = mxGetPr(prhs[1]);
  ppos   = mxGetPr(prhs[2]);
  qj     = mxGetPr(prhs[3]);
  qj_dot = mxGetPr(prhs[4]);
  vb     = mxGetPr(prhs[5]);
  refLnk = mxArrayToString(prhs[6]);

#ifdef DEBUG
  mexPrintf("qj received.\n");

  for (size_t i=0; i < numDof; i++) {
    mexPrintf(" %f", *(qj + i));
  }
#endif

  double R_rmo[9];
  reorderMatrixInRowMajor(pR, R_rmo); // matrix in "row major order"
  wbi::Rotation rot3d(R_rmo);

  wf_H_b = wbi::Frame(rot3d, ppos);

  std::string com("com");
  int refLnkID = -1; // if refLnk = "com"

  if (com.compare(refLnk) != 0) {
    robotModel->getFrameList().idToIndex(refLnk, refLnkID);
  }

  if ( !robotModel->computeDJdq(qj, wf_H_b, qj_dot, vb, refLnkID, dJdq) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI DJDq call.");
  }
  return true;
}
