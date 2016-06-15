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
// #include <string.h>

// library includes
// #include <wbi/iWholeBodyModel.h>

// local includes
#include "modelsetworldframe.h"

using namespace mexWBIComponent;

ModelSetWorldFrame *ModelSetWorldFrame::modelSetWorldFrame = 0;

ModelSetWorldFrame::ModelSetWorldFrame() : ModelComponent(3, 2, 0)
{
#ifdef DEBUG
  mexPrintf("ModelSetWorldFrame constructed\n");
#endif
}

ModelSetWorldFrame::~ModelSetWorldFrame()
{
}

ModelSetWorldFrame* ModelSetWorldFrame::getInstance()
{
  if(modelSetWorldFrame == NULL)
    modelSetWorldFrame = new ModelSetWorldFrame;

  return modelSetWorldFrame;
}

void ModelSetWorldFrame::deleteInstance()
{
  deleteObject(&modelSetWorldFrame);
}

bool ModelSetWorldFrame::allocateReturnSpace(int a, mxArray *[])
{
  // Nothing to do really
  return true;
}

bool ModelSetWorldFrame::compute(int nrhs, const mxArray *prhs[])
{
  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 ||
      mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 3 || mxGetN(prhs[3]) != 1 )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components");
  }
  double *R_temp, *p_temp, *g_temp;
  R_temp = mxGetPr(prhs[1]);
  p_temp = mxGetPr(prhs[2]);
  g_temp = mxGetPr(prhs[3]);

  double tempR[9];//, tempP[3];

  // for (int i = 0; i < 3; i++)
  // {
  //   tempP[i] = p_temp[i];
  // }
  reorderMatrixInRowMajor(R_temp, tempR);
  wbi::Rotation tempRot(tempR);
  wbi::Frame tempFrame(tempRot, p_temp); //tempP);

  modelState->setRootWorldRotoTranslation(tempFrame);
  modelState->setGravity(g_temp);

  return true;
}

bool ModelSetWorldFrame::computeFast(int nrhs, const mxArray* prhs[])
{
  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components");

  double *R_temp, *p_temp;
  R_temp = mxGetPr(prhs[1]);
  p_temp = mxGetPr(prhs[2]);

  // double tempR[9], tempP[3];
  // for (int i = 0; i < 9; i++)
  // {
  //   tempR[i] = R_temp[i];
  //   if (i < 3)
  //   {
  //     tempP[i] = p_temp[i];
  //   }
  // }

  wbi::Rotation tempRot(R_temp); //tempR);
  wbi::Frame tempFrame(tempRot, p_temp); //tempP);

  //H_baseLink_wrWorld = temp;
  //H_baseLink_wrWorld = tempFrame;

  modelState->setRootWorldRotoTranslation(tempFrame);
  //mexPrintf("Roto translation of world from base frame applied\n");

  return true;
}
