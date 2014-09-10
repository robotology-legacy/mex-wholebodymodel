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
#include <wbiIcub/icubWholeBodyModel.h>

// local includes
#include "modelcentroidalmomentum.h"

using namespace mexWBIComponent;

ModelCentroidalMomentum * ModelCentroidalMomentum::modelCentroidalMomentum;

ModelCentroidalMomentum::ModelCentroidalMomentum() : ModelComponent(3,0,1)
{
#ifdef DEBUG
  mexPrintf("ModelCentroidalMomentum constructed\n");
#endif
//   g = new double(3);
//   //g[0] = 0; g[1] = 0;g[2] = -9.81;
//   //since the foot frame is with x pointing upwards g must also be in that frame
//   g[0] = -9.81; g[1] = 0; g[2] = 0;
}

ModelCentroidalMomentum::~ModelCentroidalMomentum()
{/*
  //delete(h);
  delete(g);*/
}

ModelCentroidalMomentum * ModelCentroidalMomentum::getInstance()
{
  if(modelCentroidalMomentum == NULL)
  {
    modelCentroidalMomentum = new ModelCentroidalMomentum;
  }
  return(modelCentroidalMomentum);
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
    xB = modelState->rootRotoTrans();
    vb = modelState->vb();
   
    if(!robotModel->computeCentroidalMomentum(qj,xB,qjDot,vb,h))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI computeCentroidalMomentum call");
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
bool ModelCentroidalMomentum::processArguments(int nrhs, const mxArray* prhs[])
{
  
  if(mxGetM(prhs[1]) != numDof || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != numDof || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 6 || mxGetN(prhs[3]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state argument dimensions in ModelCentroidalMomentum call");
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

  robotModel->computeH(qj,wbi::Frame(),ROBOT_BASE_FRAME_LINK, H_base_wrfLink);
    H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
  xB.set4x4Matrix (H_w2b.data());
  
  if(h != NULL)
  {
    if(!robotModel->computeCentroidalMomentum(qj,xB,qjDot,vb,h))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the WBI computeCentroidalMomentum call");
    }
    
    
  }
}

