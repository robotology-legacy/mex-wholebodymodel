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

// local includes
#include "modelsetworldframe.h"

using namespace mexWBIComponent;

ModelSetWorldFrame *ModelSetWorldFrame::modelSetWorldFrame = 0;

ModelSetWorldFrame::ModelSetWorldFrame() : ModelComponent(3, 2, 0)
{
#ifdef DEBUG
  mexPrintf("ModelSetWorldFrame constructed.\n");
#endif
}

ModelSetWorldFrame::~ModelSetWorldFrame()
{
}

ModelSetWorldFrame* ModelSetWorldFrame::getInstance()
{
  if (modelSetWorldFrame == 0) {
    modelSetWorldFrame = new ModelSetWorldFrame;
  }
  return modelSetWorldFrame;
}

void ModelSetWorldFrame::deleteInstance()
{
  deleteObject(&modelSetWorldFrame);
}

bool ModelSetWorldFrame::allocateReturnSpace(int nlhs, mxArray **plhs)
{
  // Nothing to do really
  return true;
}

bool ModelSetWorldFrame::compute(int nrhs, const mxArray **prhs)
{
  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 ||
      mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 3 || mxGetN(prhs[3]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components.");
  }
  double *pR, *ppos, *pg;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);
  pg   = mxGetPr(prhs[3]);

  setWorldFrame(pR, ppos);
  modelState->setGravity(pg);
  return true;
}

bool ModelSetWorldFrame::computeFast(int nrhs, const mxArray **prhs)
{
  if ( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components.");
  }
  double *pR, *ppos;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);

  setWorldFrame(pR, ppos);
  return true;
}

void ModelSetWorldFrame::setWorldFrame(const double *pR, const double *ppos)
{
  double R_rmo[9];
  reorderMatrixInRowMajor(pR, R_rmo); // matrix in "row major order"

  wbi::Rotation rot3d(R_rmo);
  wbi::Frame wf_H_b(rot3d, ppos);

  modelState->setBase2WorldTransformation(wf_H_b);
}
