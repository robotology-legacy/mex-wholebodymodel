/*
 * Copyright (C) 2016 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Martin Neururer
 * email: martin.neururer@gmail.com, gabriele.nava@iit.it
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
#include "modeltransformationmatrix.h"

using namespace mexWBIComponent;

ModelTransformationMatrix *ModelTransformationMatrix::modelTransformationMatrix = 0;

double *ModelTransformationMatrix::qj       = 0;
char   *ModelTransformationMatrix::refLnk   = 0;
double *ModelTransformationMatrix::wf_H_lnk = 0;

ModelTransformationMatrix::ModelTransformationMatrix() : ModelComponent(4,1,1)
{
#ifdef DEBUG
  mexPrintf("ModelTransformationMatrix constructed.\n");
#endif
}

ModelTransformationMatrix::~ModelTransformationMatrix()
{
}

bool ModelTransformationMatrix::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelTransformationMatrix.\n");
#endif
  plhs[0] = mxCreateDoubleMatrix(4, 4, mxREAL);
  wf_H_lnk = mxGetPr(plhs[0]);

  return true;
}

ModelTransformationMatrix* ModelTransformationMatrix::getInstance()
{
  if (modelTransformationMatrix == 0) {
    modelTransformationMatrix = new ModelTransformationMatrix;
  }
  return modelTransformationMatrix;
}

void ModelTransformationMatrix::deleteInstance()
{
  deleteObject(&modelTransformationMatrix);
}

bool ModelTransformationMatrix::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelTransformationMatrix performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelTransformationMatrix::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelTransformationMatrix performing computeFast.\n");
#endif
#ifdef DEBUG
  if (wf_H_lnk == 0) {
    return false;
  }
#endif
  if( !mxIsChar(prhs[1]) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed component in ModelTransformationMatrix call.");
  }
  robotModel = modelState->robotModel();

  std::string strCom = "com";
  int refLnkID = -1; // the ID for ref. link "com" is -1

  refLnk = mxArrayToString(prhs[1]);
  if (strCom.compare(refLnk) != 0) { // if refLnk != "com"
    // get the index number from the frame list ...
    robotModel->getFrameList().idToIndex(refLnk, refLnkID);
  }

  wf_H_b = modelState->getBase2WorldTransformation();
  qj     = modelState->qj();

  wbi::Frame frm3d_H;
  if ( !robotModel->computeH(qj, wf_H_b, refLnkID, frm3d_H) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the computeH call.");
  }

  double H_rmo[16]; // transformation matrix in "row major order"
  frm3d_H.get4x4Matrix(H_rmo);
  // since the values in the array are stored in row-major order and Matlab
  // uses the column-major order for multi-dimensional arrays, we have to
  // make an array-transposition ...
  reorderMatrixInColMajor(H_rmo, wf_H_lnk, 4, 4);

  return true;
}

bool ModelTransformationMatrix::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (wf_H_lnk == 0) {
    return false;
  }
#endif
  size_t numDof = modelState->dof();

  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
       mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || !mxIsChar(prhs[4]) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions/components in ModelTransformationMatrix call.");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR     = mxGetPr(prhs[1]);
  ppos   = mxGetPr(prhs[2]);
  qj     = mxGetPr(prhs[3]);
  refLnk = mxArrayToString(prhs[4]);

#ifdef DEBUG
  mexPrintf("qj received.\n");

  for (size_t i=0; i < numDof; i++) {
    mexPrintf(" %f", *(qj + i));
  }
#endif

  double R_rmo[9];
  reorderMatrixInRowMajor(pR, R_rmo);
  wbi::Rotation rot3d(R_rmo);

  wf_H_b = wbi::Frame(rot3d, ppos);

  std::string strCom = "com";
  int refLnkID = -1; // if ref. link = "com"

  if (strCom.compare(refLnk) != 0) { // if refLnk != "com"
    robotModel->getFrameList().idToIndex(refLnk, refLnkID);
  }

  wbi::Frame frm3d_H;
  if ( !robotModel->computeH(qj, wf_H_b, refLnkID, frm3d_H) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the computeH call.");
  }

  double H_rmo[16]; // transformation matrix in "row major order"
  frm3d_H.get4x4Matrix(H_rmo);
  reorderMatrixInColMajor(H_rmo, wf_H_lnk, 4, 4); // put the output matrix in "column major order"

  return true;
}
