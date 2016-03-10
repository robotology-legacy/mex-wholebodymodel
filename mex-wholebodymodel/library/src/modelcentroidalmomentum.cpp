/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 *  Authors: Naveen Kuppuswamy
 *  email: naveen.kuppuswamy@iit.it
 *
 *  The development of this software was supported by the FP7 EU projects
 *  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 *  http://www.codyco.eu
 *
 *  Permission is granted to copy, distribute, and/or modify this program
 *  under the terms of the GNU General Public License, version 2 or any
 *  later version published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details
 *
 *
 */

// global includes

// library includes
#include <wbi/iWholeBodyModel.h>
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>

// local includes
#include "modelcentroidalmomentum.h"

using namespace mexWBIComponent;

ModelCentroidalMomentum * ModelCentroidalMomentum::modelCentroidalMomentum;

ModelCentroidalMomentum::ModelCentroidalMomentum() : ModelComponent(5,0,1)
{
#ifdef DEBUG
  mexPrintf("ModelCentroidalMomentum constructed\n");
#endif
}

ModelCentroidalMomentum::~ModelCentroidalMomentum()
{
}

ModelCentroidalMomentum * ModelCentroidalMomentum::getInstance()
{
  if(modelCentroidalMomentum == NULL)
  {
    modelCentroidalMomentum = new ModelCentroidalMomentum;
  }
  return(modelCentroidalMomentum);
}

void ModelCentroidalMomentum::deleteInstance()
{
  deleteObject(&modelCentroidalMomentum);
}


bool ModelCentroidalMomentum::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelCentroidalMomentum\n");
#endif

  plhs[0] =  mxCreateNumericMatrix(6, 1, mxDOUBLE_CLASS, mxREAL);
  h = mxGetPr(plhs[0]);
  return(true);
}

bool ModelCentroidalMomentum::compute(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute generalised bias forces\n");
#endif
  processArguments(nrhs,prhs);
  return(true);
}

bool ModelCentroidalMomentum::computeFast(int nrhs, const mxArray* prhs[])
{

#ifdef DEBUG
   mexPrintf("Trying to fast compute generalised bias forces\n");
#endif
   robotModel = modelState->robotModel();

    qj = modelState->qj();
    qjDot = modelState->qjDot();
    world_H_rootLink = modelState->getRootWorldRotoTranslation();
    vb = modelState->vb();

    if(!robotModel->computeCentroidalMomentum(qj,world_H_rootLink,qjDot,vb,h))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI computeCentroidalMomentum call");
    }
    return(true);
}

bool ModelCentroidalMomentum::processArguments(int nrhs, const mxArray* prhs[])
{
  size_t numDof = modelState->dof();

  if(mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != numDof || mxGetN(prhs[4]) != 1 || mxGetM(prhs[5]) != 6 || mxGetN(prhs[5]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state argument dimensions in ModelCentroidalMomentum call");
  }
  robotModel = modelState->robotModel();
  qj = mxGetPr(prhs[3]);
  qjDot = mxGetPr(prhs[4]);
  vb = mxGetPr(prhs[5]);
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

  for(size_t i = 0; i< numDof; i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif
  world_H_rootLink = tempFrame;

  if(h != NULL)
  {
    if(!robotModel->computeCentroidalMomentum(qj,world_H_rootLink,qjDot,vb,h))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI computeCentroidalMomentum call");
    }


  }

  return true;
}

