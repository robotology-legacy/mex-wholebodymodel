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
#include "modelforwardkinematics.h"

using namespace mexWBIComponent;

ModelForwardKinematics *ModelForwardKinematics::modelForwardKinematics = 0;

double *ModelForwardKinematics::qj      = 0;
char   *ModelForwardKinematics::refLnk  = 0;
double *ModelForwardKinematics::vqT_lnk = 0;

ModelForwardKinematics::ModelForwardKinematics() : ModelComponent(4, 1, 1)
{
}

ModelForwardKinematics::~ModelForwardKinematics()
{
#ifdef DEBUG
  mexPrintf("ModelForwardKinematics destructed.\n");
#endif
}

ModelForwardKinematics *ModelForwardKinematics::getInstance()
{
  if (modelForwardKinematics == 0) {
    modelForwardKinematics = new ModelForwardKinematics();
  }
  return modelForwardKinematics;
}

void ModelForwardKinematics::deleteInstance()
{
  deleteObject(&modelForwardKinematics);
}

bool ModelForwardKinematics::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelForwardKinematics.\n");
#endif
  plhs[0] = mxCreateDoubleMatrix(7, 1, mxREAL);
  vqT_lnk = mxGetPr(plhs[0]);

  return true;
}
bool ModelForwardKinematics::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelForwardKinematics performing compute.\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelForwardKinematics::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if (vqT_lnk == 0) {
    return false;
  }
#endif
  size_t nDof = modelState->dof();

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

  computeForwardKin();
  return true;
}

bool ModelForwardKinematics::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("ModelForwardKinematics performing computeFast.\n");
#endif
#ifdef DEBUG
  if (vqT_lnk == 0) {
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

  computeForwardKin();

#ifdef DEBUG
  mexPrintf("ModelForwardKinematics fast computed.\n");
#endif
  return true;
}

void ModelForwardKinematics::computeForwardKin()
{
  std::string strCom("com");
  int refLnkID = -1; // if refLnk = "com"

  if (strCom.compare(refLnk) != 0) {
    if ( !robotModel->getFrameList().idToIndex(refLnk, refLnkID) ) {
      mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "forwardKinematics call: Link ID does not exist.");
    }
  }

  double vxT_lnk[7]; // vector axis-angle transformation (from ref. link to world frame)
  if ( !(robotModel->forwardKinematics(qj, wf_H_b, refLnkID, vxT_lnk)) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the WBI forwardKinematics call.");
  }

  double axang[4];
  memcpy(axang, (vxT_lnk + 3), sizeof(double)*4); // axis-angle vector

#ifdef DEBUG
  std::stringstream ssR;
  ssR << "axis-angle: [" << vxT_lnk[3] << "," << vxT_lnk[4] << "," << vxT_lnk[5] << "," << vxT_lnk[6] << "]\n";
  std::string sR = ssR.str();
  mexPrintf(sR.c_str());
  mexPrintf("rotation:\n");
  mexPrintf((rot3d.toString()).c_str());
#endif

  wbi::Rotation rot3d = wbi::Rotation::axisAngle(axang);

  double quat_lnk[4];
  rot3d.getQuaternion(quat_lnk[1], quat_lnk[2], quat_lnk[3], quat_lnk[0]);

  memcpy(vqT_lnk, vxT_lnk, sizeof(double)*3);        // copy position vector
  memcpy((vqT_lnk + 3), quat_lnk, sizeof(double)*4); // copy quaternion
}
