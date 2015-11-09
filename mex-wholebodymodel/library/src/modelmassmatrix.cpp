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
// #include <wbiIcub/icubWholeBodyModel.h>
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>

//local includes
#include "modelmassmatrix.h"

using namespace mexWBIComponent;

ModelMassMatrix * ModelMassMatrix::modelMassMatrix = 0;

ModelMassMatrix::ModelMassMatrix(): ModelComponent(3,0,1)
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix constructed \n");
#endif
}

ModelMassMatrix::~ModelMassMatrix()
{
#ifdef DEBUG
  mexPrintf("ModelMassMatrix destructed\n");
#endif
}

bool ModelMassMatrix::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelMassMatrix\n");
#endif
  bool returnVal = false;

  int numDof = modelState->dof();

  plhs[0]=mxCreateDoubleMatrix(numDof+6,numDof+6, mxREAL);

  massMatrix = mxGetPr(plhs[0]);
  returnVal = true;
  return(returnVal);
}

ModelMassMatrix * ModelMassMatrix::getInstance()
{
  if(modelMassMatrix == NULL)
  {
    modelMassMatrix = new ModelMassMatrix;
  }
  return(modelMassMatrix);
}

void ModelMassMatrix::deleteInstance()
{
  deleteObject(&modelMassMatrix);
}


bool ModelMassMatrix::compute(int nrhs, const mxArray * prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelMassMatrix \n");
#endif
  processArguments(nrhs,prhs);
#ifdef DEBUG
  mexPrintf("ModelMassMatrix computed\n");
#endif
  return(true);
}

bool ModelMassMatrix::computeFast(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelMassMatrix \n");
#endif
  robotModel = modelState->robotModel();
  qj = modelState->qj();
  world_H_rootLink = modelState->getRootWorldRotoTranslation();

  if(!robotModel->computeMassMatrix(qj,world_H_rootLink,massMatrix))
  {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI MassMatrix call");
  }


#ifdef DEBUG
  mexPrintf("ModelMassMatrix computed\n");
#endif
  return(true);

}



bool ModelMassMatrix::processArguments(int nrhs, const mxArray * prhs[])
{
  size_t numDof = modelState->dof();

  if(mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions");
  }
  robotModel = modelState->robotModel();
  qj = mxGetPr(prhs[3]);
  double *R_temp,*p_temp;
  R_temp = (double *)mxGetPr(prhs[1]);
  p_temp = (double *)mxGetPr(prhs[2]);

  double tempR[9],tempP[3];
  for(int i = 0;i<3;i++)
  {
     tempP[i] = p_temp[i];
  }
  
  reorderMatrixElements(R_temp, tempR);
  wbi::Rotation tempRot(tempR);
  wbi::Frame tempFrame(tempRot, tempP);

#ifdef DEBUG
  mexPrintf("qj received \n");
  for(size_t i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif
  world_H_rootLink = tempFrame;

  if(massMatrix != NULL)
  {
     if(!robotModel->computeMassMatrix(qj,world_H_rootLink,massMatrix))
     {
	mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI MassMatrix call");
     }
  }
  return(true);
}




















