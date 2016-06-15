/*
 * Copyright (C) 2016 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Martin Neururer
 * email: martin.neururer@gmail.com, gabriele.nava@iit.it
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

//library includes
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>

//local includes
#include "modelrototranslationmatrix.h"

using namespace mexWBIComponent;

ModelRotoTranslationMatrix *ModelRotoTranslationMatrix::modelRotoTranslationMatrix;

ModelRotoTranslationMatrix::ModelRotoTranslationMatrix() : ModelComponent(4,1,1)
{
#ifdef DEBUG
  mexPrintf("ModelRotoTranslationMatrix constructed\n");
#endif
}

ModelRotoTranslationMatrix::~ModelRotoTranslationMatrix()
{
}

bool ModelRotoTranslationMatrix::allocateReturnSpace(int nlhs, mxArray **plhs)
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelRotoTranslationMatrix\n");
#endif
  plhs[0] = mxCreateDoubleMatrix(4, 4, mxREAL);
  H = mxGetPr(plhs[0]);

  return true;
}

ModelRotoTranslationMatrix* ModelRotoTranslationMatrix::getInstance()
{
  if(modelRotoTranslationMatrix == NULL)
    modelRotoTranslationMatrix = new ModelRotoTranslationMatrix;

  return modelRotoTranslationMatrix;
}

void ModelRotoTranslationMatrix::deleteInstance()
{
  deleteObject(&modelRotoTranslationMatrix);
}

bool ModelRotoTranslationMatrix::compute(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelRotoTranslationMatrix\n");
#endif
  return processArguments(nrhs, prhs);
}

bool ModelRotoTranslationMatrix::computeFast(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to fast compute ModelRotoTranslationMatrix\n");
#endif
#ifdef DEBUG
  if(H == NULL) return false;
#endif
  if(!mxIsChar(prhs[1]))
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed component in ModelRotoTranslationMatrix call");

  robotModel = modelState->robotModel();
  std::string strCom = "com";
  int refLinkID = -1; // ID for ref. link "com" is -1

  strRefLink = mxArrayToString(prhs[1]);
  if(strCom.compare(strRefLink) != 0) // if strRefLink != "com"
  {
    // get the index number from the frame list ...
    robotModel->getFrameList().idToIndex(strRefLink, refLinkID);
  }

  qj = modelState->qj();
  world_H_rootLink = modelState->getRootWorldRotoTranslation();

  wbi::Frame w_H_reflnk;
  if( !robotModel->computeH(qj, world_H_rootLink, refLinkID, w_H_reflnk) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the computeH call");

  double H_arr[16];
  w_H_reflnk.get4x4Matrix(H_arr);

  // since the values in the array are stored in row-major order and Matlab
  // uses the column-major order for multi-dimensional arrays, we have to
  // make an array-transposition ...
  reorderMatrixInColMajor(H_arr, H, 4, 4);
  return true;
}

bool ModelRotoTranslationMatrix::processArguments(int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  if(H == NULL) return false;
#endif
  size_t numDof = modelState->dof();

  if( mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 ||
      mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || !mxIsChar(prhs[4]) )
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state argument dimensions/components in ModelRotoTranslationMatrix call");
  }
  robotModel = modelState->robotModel();

  double *pR, *ppos;
  pR   = mxGetPr(prhs[1]);
  ppos = mxGetPr(prhs[2]);

  qj         = mxGetPr(prhs[3]);
  strRefLink = mxArrayToString(prhs[4]);

#ifdef DEBUG
  mexPrintf("qj received\n");

  for(size_t i=0; i < numDof; i++)
    mexPrintf(" %f", *(qj + i));
#endif

  double R_ro[9];
  reorderMatrixInRowMajor(pR, R_ro);

  wbi::Rotation rotm(R_ro);
  world_H_rootLink = wbi::Frame(rotm, ppos);

  std::string strCom = "com";
  int refLinkID = -1; // the ID for ref. link "com" is -1

  if(strCom.compare(strRefLink) != 0) // if strRefLink != "com"
    robotModel->getFrameList().idToIndex(strRefLink, refLinkID);

  wbi::Frame w_H_reflnk;
  if( !robotModel->computeH(qj, world_H_rootLink, refLinkID, w_H_reflnk) )
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Something failed in the computeH call");

  double H_arr[16];
  w_H_reflnk.get4x4Matrix(H_arr);

  reorderMatrixInColMajor(H_arr, H, 4, 4);
  return true;
}
