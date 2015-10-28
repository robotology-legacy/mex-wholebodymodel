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

// local includes
#include "modelgetstate.h"
#include <boost/concept_check.hpp>

using namespace mexWBIComponent;

ModelGetState *ModelGetState::modelGetState;


ModelGetState::ModelGetState() : ModelComponent(0,1,4)
{
#ifdef DEBUG
  mexPrintf("ModelGetState Constructuted\n");
#endif
}

ModelGetState::~ModelGetState()
{

}

ModelGetState* ModelGetState::getInstance()
{
  if(modelGetState == NULL)
  {
    modelGetState = new ModelGetState;
  }
  return(modelGetState);
}

void ModelGetState::deleteInstance()
{
  deleteObject(&modelGetState);
}



bool ModelGetState::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
  #ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGetState\n");
#endif

  bool returnVal = false;
  if(nlhs!=4)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumOutputs","4 output arguments required for ModelGetState");
  }

//   if(mxGetM(plhs[0]) != numDof || mxGetN(plhs[0]) != 1 || mxGetM(plhs[1]) != 7 || mxGetN(plhs[1]) != 1 )
//   {
//      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
//   }

  int numDof = modelState->dof();


    //p[0] =  mxCreateNumericMatrix(numDof, 1, mxDOUBLE_CLASS, mxREAL);//
  plhs[0]=mxCreateDoubleMatrix(numDof,1, mxREAL); //qj
  plhs[1]=mxCreateDoubleMatrix(7,1, mxREAL); //wTb
  plhs[2]=mxCreateDoubleMatrix(numDof,1, mxREAL); //qjDot
  plhs[3]=mxCreateDoubleMatrix(6,1, mxREAL); //vb

  returnVal = true;

  qj = mxGetPr(plhs[0]);
  wTb = mxGetPr(plhs[1]);
  qjDot = mxGetPr(plhs[2]);
  vb = mxGetPr(plhs[3]);

  returnVal = true;
  return(returnVal);
}

bool ModelGetState::compute(int nrhs, const mxArray* prhs[])
{
  int i;

#ifdef DEBUG
  mexPrintf("ModelGetState performing Compute");
#endif
  //double *qjTemp;//qjTemp[numDof];

  //qj = modelState->qj();
  modelState->qj(qj);
//  qjDot = modelState->qjDot();
//  vb = modelState->vb();
  //rootRotoTrans = modelState->rootRotoTrans();
//   world_H_rootLink = modelState->computeRootWorldRotoTranslation(qj);
  world_H_rootLink = modelState->getRootWorldRotoTranslation();
//   modelState->computeRootWorldRotoTranslation(qj);

#ifdef DEBUG

  mexPrintf("Inside getState\nRootWorldRotoTrans\n");

  mexPrintf("world_H_root\n");
  mexPrintf((world_H_rootLink.R.toString()).c_str());
  mexPrintf(" = \n");

  mexPrintf("world_H_ReferenceLink\n");
  mexPrintf(( (modelState->getReferenceToWorldFrameRotoTrans()).R.toString()).c_str());

  mexPrintf("\n\n");

#endif

  world_H_rootLink.R.getQuaternion(rootQuaternion[1], rootQuaternion[2], rootQuaternion[3], rootQuaternion[0]);

#ifdef DEBUG
      std::stringstream ssR;
    ssR<<"Quat : ["<<rootQuaternion[0]<<","<<rootQuaternion[1]<<","<<rootQuaternion[2]<<","<<rootQuaternion[3]<<"]\n";
    std::string sR = ssR.str();

    mexPrintf(sR.c_str());
#endif

    for (i = 0; i<3 ; i++)
  {
    wTb[i] = world_H_rootLink.p[i];
  }
  for (i=0 ; i<4;i++)
  {
    wTb[3+i] = rootQuaternion[i];
  }

 // qjDot = modelState->qjDot();
 // vb = modelState->vb();
  modelState->qjDot(qjDot);
  modelState->vb(vb);
  /*
  for(int i = 0 ; i< numDof; i++)
  {
    mexPrintf("ModelGetState qj[%d] : %2.2f, qjDot[%d] : %2.2f\n",i,qj[i],i,qjDot[i]);
  }
  */

  return true;
}

bool ModelGetState::computeFast(int nrhs, const mxArray* prhs[])
{
  int i;

#ifdef DEBUG
  mexPrintf("ModelGetState performing Compute");
#endif

  //qj = modelState->qj();
  modelState->qj(qj);
  //rootRotoTrans = computeRootWorldRotoTranslation(qj);

 // world_H_rootLink = computeRootWorldRotoTranslation(qj);

//  world_H_rootLink = modelState->computeRootWorldRotoTranslation();
  world_H_rootLink = modelState->getRootWorldRotoTranslation();
#ifdef DEBUG
  mexPrintf("Inside getState\nRootWorldRotoTrans\n");

  mexPrintf("world_H_root\n");
  mexPrintf((world_H_rootLink.R.toString()).c_str());
  mexPrintf(" = \n");

  mexPrintf("world_H_ReferenceLink\n");
  mexPrintf(( (modelState->getReferenceToWorldFrameRotoTrans()).R.toString()).c_str());

  mexPrintf("\n\n");
  //rootQuaternion =
#endif

  (modelState->getRootWorldRotoTranslation()).R.getQuaternion(rootQuaternion[1], rootQuaternion[2], rootQuaternion[3], rootQuaternion[0]);
#ifdef DEBUG
  //wbi::Rotation::getQuaternion(rootRotoTrans.R);

      std::stringstream ssR;
    ssR<<"Quat : ["<<rootQuaternion[0]<<","<<rootQuaternion[1]<<","<<rootQuaternion[2]<<","<<rootQuaternion[3]<<"]\n";
    std::string sR = ssR.str();
    mexPrintf(sR.c_str());
#endif

  for (i = 0; i<3 ; i++)
  {
    wTb[i] = world_H_rootLink.p[i];
  }
  for (i=0 ; i<4;i++)
  {
    wTb[3+i] = rootQuaternion[i];
  }

  //qjDot = modelState->qjDot();
  //vb = modelState->vb();

  modelState->qjDot(qjDot);
  modelState->vb(vb);
  /*

  for(int i = 0 ; i< numDof; i++)
  {
    mexPrintf("ModelGetState qj[%d] : %2.2f, qjDot[%d] : %2.2f\n",i,qj[i],i,qjDot[i]);
  }
  */
    return true;
}

