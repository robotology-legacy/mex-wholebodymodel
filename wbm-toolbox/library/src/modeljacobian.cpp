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
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>

// local includes
#include "modeljacobian.h"

using namespace mexWBIComponent;

ModelJacobian *ModelJacobian::modelJacobian = 0;

size_t  ModelJacobian::nDof     = 0;
int     ModelJacobian::nCols    = 0;
double *ModelJacobian::qj       = 0;
char   *ModelJacobian::refLnk   = 0;
double *ModelJacobian::J_rmo    = 0;
double *ModelJacobian::wf_J_lnk = 0;

ModelJacobian::ModelJacobian() : ModelComponent(4, 1, 1)
{
#ifdef DEBUG
  mexPrintf("ModelJacobian constructed.\n");
#endif
}

ModelJacobian::~ModelJacobian()
{
  if (J_rmo != 0) {
    delete[] J_rmo;
    J_rmo = 0;
  }
}

bool ModelJacobian::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelJacobian.\n");
#endif
  nDof = modelState->dof();
  nCols  = (int)nDof + 6;

  plhs[0] = mxCreateDoubleMatrix(6, nCols, mxREAL);
  wf_J_lnk = mxGetPr(plhs[0]);

  if (J_rmo == 0) {
    J_rmo = new double[6*nCols];
  }
  return true;
}

ModelJacobian *ModelJacobian::getInstance()
{
  if (modelJacobian == 0) {
    modelJacobian = new ModelJacobian;
  }
  return modelJacobian;
}

void ModelJacobian::deleteInstance()
{
  deleteObject(&modelJacobian);
}

bool ModelJacobian::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelJacobian performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelJacobian::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelJacobian performing computeFast.\n");
#endif
#ifdef DEBUG
  if ( (J_rmo == 0) || (wf_J_lnk == 0) ) {
    return false;
  }
#endif
  if ( !mxIsChar(prhs[1]) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components.");
  }
  robotModel = modelState->robotModel();

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();
  refLnk = mxArrayToString(prhs[1]);

  int refLnkID = getRefLinkID();

  if ( !(robotModel->computeJacobian(qj, wf_H_b, refLnkID, J_rmo)) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeJacobian call.");
  }
  // since Matlab uses the column-major order for multi-dimensional arrays,
  // we have to make an array-transposition ...
  reorderMatrixInColMajor(J_rmo, wf_J_lnk, 6, nCols);

#ifdef DEBUG
  mexPrintf("ModelJacobian fast computed.\n");
#endif
  return true;
}

bool ModelJacobian::processArguments(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  if ( (J_rmo == 0) || (wf_J_lnk == 0) ) {
    return false;
  }
#endif
  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
       mxGetM(prhs[3]) != nDof || mxGetN(prhs[3]) != 1 || !mxIsChar(prhs[4]) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components.");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR     = mxGetPr(prhs[1]);
  ppos   = mxGetPr(prhs[2]);
  qj     = mxGetPr(prhs[3]);
  refLnk = mxArrayToString(prhs[4]);

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

  int refLnkID = getRefLinkID();

  if ( !(robotModel->computeJacobian(qj, wf_H_b, refLnkID, J_rmo)) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI computeJacobian call.");
  }
  reorderMatrixInColMajor(J_rmo, wf_J_lnk, 6, nCols); // put the output matrix in "column major order"

  return true;
}

int ModelJacobian::getRefLinkID()
{
  std::string strCom("com");
  int refLnkID = -1; // if refLnk = "com"

  if (strCom.compare(refLnk) != 0) {
    if ( !robotModel->getFrameList().idToIndex(refLnk, refLnkID) ) {
      mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "jacobian call: Link ID does not exist.");
    }
  }
  return refLnkID;
}
