/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
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
#include <wbi/iWholeBodyModel.h>
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>

//local includes
#include <Eigen/Core>

#include "modeljacobian.h"
using namespace mexWBIComponent;

ModelJacobian * ModelJacobian::modelJacobian;

ModelJacobian::ModelJacobian(): ModelComponent(4,1,1)
{
    j_rowMajor = NULL;
#ifdef DEBUG
  mexPrintf("ModelJacobian constructed \n");
#endif
}

ModelJacobian::~ModelJacobian()
{

    if(j_rowMajor != NULL){
        free(j_rowMajor);
            j_rowMajor = 0;
    }
}

bool ModelJacobian::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelJacobian\n");
#endif
  int numDof = modelState->dof();

  bool returnVal = false;

  plhs[0]=mxCreateDoubleMatrix(6,numDof+6, mxREAL);
  j_colMajor = mxGetPr(plhs[0]);

    if(j_rowMajor == NULL)
  {
      j_rowMajor = (double*)malloc(sizeof(double) * 6 * (numDof + 6));
  }
  returnVal = true;
  return(returnVal);
}

ModelJacobian * ModelJacobian::getInstance()
{
  if(modelJacobian == NULL)
  {
    modelJacobian = new ModelJacobian;
  }
  return(modelJacobian);
}

void ModelJacobian::deleteInstance()
{
  deleteObject(&modelJacobian);
}


bool ModelJacobian::compute(int nrhs, const mxArray * prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelJacobian \n");
#endif
  processArguments(nrhs,prhs);
#ifdef DEBUG
  mexPrintf("ModelJacobian computed\n");
#endif
  return(true);
}

bool ModelJacobian::computeFast(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to fastCompute ModelJacobian \n");
#endif

  if(!mxIsChar(prhs[1]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }

  int numDof = modelState->dof();

  qj = modelState->qj();
//   world_H_rootLink = modelState->computeRootWorldRotoTranslation(qj);
  world_H_rootLink = modelState->getRootWorldRotoTranslation();
  refLink = mxArrayToString(prhs[1]);
  robotModel = modelState->robotModel();
  int refLinkID;
  std::string com("com");

  if(com.compare(refLink)==0)
  {
    refLinkID = -1;
  }
  else
  {
    //robotModel->getLinkId (refLink, refLinkID);
    robotModel->getFrameList().idToIndex(refLink, refLinkID);
  }

    modelState = ModelState::getInstance();

  if(!(robotModel->computeJacobian(qj,world_H_rootLink,refLinkID,j_rowMajor)))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the jacobian call");
  }
  int columnMajCtr = 0;
  for (int i = 0; i<(numDof+6); i++)
  {
      for (int j = 0;j<6;j++)
      {
          j_colMajor[columnMajCtr] = j_rowMajor[i+j*(numDof+6)];
          columnMajCtr = columnMajCtr+1;
      }
  }
#ifdef DEBUG
  mexPrintf("ModelJacobian fastComputed\n");
#endif
  return(true);
}

bool ModelJacobian::processArguments(int nrhs, const mxArray * prhs[])
{
  size_t numDof = modelState->dof();

  if(mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || !mxIsChar(prhs[4]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
  robotModel = modelState->robotModel();

  qj = mxGetPr(prhs[3]);
  refLink = mxArrayToString(prhs[4]);

  double *R_temp,*p_temp;
  R_temp = (double *)mxGetPr(prhs[1]);
  p_temp = (double *)mxGetPr(prhs[2]);

  double tempR[9],tempP[3];
  for(int i = 0;i<9;i++)
  {
    tempR[i] = R_temp[i];
    if(i<3)
     {
       tempP[i] = p_temp[i];
     }
  }
  wbi::Rotation tempRot(tempR);
  wbi::Frame tempFrame(tempRot, tempP);

#ifdef DEBUG
  mexPrintf("qj received \n");

  for(size_t i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif

  world_H_rootLink = tempFrame;//modelState->computeRootWorldRotoTranslation(qj);

  if(j_rowMajor != NULL && j_colMajor != NULL)
  {
    int refLinkID;
    std::string com("com");

    if(com.compare(refLink)==0)
    {
      refLinkID = -1;
    }
    else
    {
      robotModel->getFrameList().idToIndex(refLink, refLinkID);
    }
    if(!(robotModel->computeJacobian(qj,world_H_rootLink,refLinkID,j_rowMajor)))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the jacobian call");
    }
  }
  int columnMajCtr = 0;
  for (size_t i = 0; i<(numDof+6); i++)
  {
      for (int j = 0;j<6;j++)
      {
          j_colMajor[columnMajCtr] = j_rowMajor[i+j*(numDof+6)];
          columnMajCtr = columnMajCtr+1;
      }
  }

  return(true);
}
