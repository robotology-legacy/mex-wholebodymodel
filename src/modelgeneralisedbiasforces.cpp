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
#include "modelgeneralisedbiasforces.h"

using namespace mexWBIComponent;

ModelGeneralisedBiasForces * ModelGeneralisedBiasForces::modelGeneralisedBiasForces;

ModelGeneralisedBiasForces::ModelGeneralisedBiasForces() : ModelComponent(3,0,1)
{
#ifdef DEBUG
  mexPrintf("ModelGeneralisedBiasForces constructed\n");
#endif
  g = modelState->g();
  //g[0] = 0; g[1] = 0;g[2] = -9.81;
  //since the foot frame is with x pointing upwards g must also be in that frame
  /*
  qj = new double(numDof);
  qjDot = new double(numDof);
  vb = new double(6);
  */
}

ModelGeneralisedBiasForces::~ModelGeneralisedBiasForces()
{
  //delete(h);
  delete(g);
}

ModelGeneralisedBiasForces * ModelGeneralisedBiasForces::getInstance()
{
  if(modelGeneralisedBiasForces == NULL)
  {
    modelGeneralisedBiasForces = new ModelGeneralisedBiasForces;
  }
  return(modelGeneralisedBiasForces);
}

bool ModelGeneralisedBiasForces::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGeneralisedBiasForces\n");
#endif

  plhs[0] =  mxCreateNumericMatrix(numDof+6, 1, mxDOUBLE_CLASS, mxREAL);
  h = mxGetPr(plhs[0]);
  return(true);
}

bool ModelGeneralisedBiasForces::compute(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute generalised bias forces\n");
#endif
  processArguments(nrhs,prhs);
  return(true);
}

bool ModelGeneralisedBiasForces::computeFast(int nrhs, const mxArray* prhs[])
{
  
#ifdef DEBUG
   mexPrintf("Trying to fast compute generalised bias forces\n");
#endif
   robotModel = modelState->robotModel();

    qj = modelState->qj();
    qjDot = modelState->qjDot();
    //xB = modelState->rootRotoTrans();
    world_H_rootLink = modelState->computeRootWorldRotoTranslation(qj);
    vb = modelState->vb();
   
    if(!robotModel->computeGeneralizedBiasForces(qj,world_H_rootLink,qjDot,vb,g,h))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI computeGeneralizedBiasForces call");
    }
    return(true);
}


/*
bool ModelGeneralisedBiasForces::display(int nrhs, const mxArray* prhs[])
{
  mexPrintf("Trying to display generalised bias forces, h:\n");
  
  double tempH[modelState->dof()+6];
  robotModel->computeGeneralizedBiasForces(modelState->q(),modelState->baseFrame(),modelState->dq(),modelState->dxb(),g,tempH);
  */
  
//   for (int i = 0;i<modelState->dof()+6;i++)
//   {
//     mexPrintf("%f ",tempH[i]);
//   }
//   return(true);
// }
bool ModelGeneralisedBiasForces::processArguments(int nrhs, const mxArray* prhs[])
{
  
  if(mxGetM(prhs[1]) != numDof || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != numDof || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 6 || mxGetN(prhs[3]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state argument dimensions in ModelGeneralisedForces call");
  }
  robotModel = modelState->robotModel();
  qj = mxGetPr(prhs[1]);
  qjDot = mxGetPr(prhs[2]);
  vb = mxGetPr(prhs[3]);
  
#ifdef DEBUG
  mexPrintf("qj received \n");

  for(int i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif  

  world_H_rootLink = modelState->computeRootWorldRotoTranslation(qj);
  
  if(h != NULL)
  {
    if(!robotModel->computeGeneralizedBiasForces(qj,world_H_rootLink,qjDot,vb,g,h))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI computeGeneralizedBiasForces call");
    }
    
    
  }
}

