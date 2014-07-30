/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2014  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include <wbi/iWholeBodyModel.h>
#include "modelgeneralisedbiasforces.h"

using namespace mexWBIComponent;

ModelGeneralisedBiasForces * ModelGeneralisedBiasForces::modelGeneralisedBiasForces;

ModelGeneralisedBiasForces::ModelGeneralisedBiasForces(wbi::iWholeBodyModel * m) : ModelComponent(m), numReturnArguments(1)
{
  mexPrintf("ModelGeneralisedBiasForces constructed\n");
  //numDof = robotModel->getDoFs();
  
//   h = new double(robotModel->getDoFs()+6);
  g = new double(3);
  g[0] = 0; g[1] = 0;g[2] = -9.81;
}

ModelGeneralisedBiasForces::~ModelGeneralisedBiasForces()
{
  delete(h);
  delete(g);
}

ModelGeneralisedBiasForces * ModelGeneralisedBiasForces::getInstance(wbi::iWholeBodyModel * m)
{
  if(modelGeneralisedBiasForces == NULL)
  {
    modelGeneralisedBiasForces = new ModelGeneralisedBiasForces(m);
  }
  return(modelGeneralisedBiasForces);
}

const int ModelGeneralisedBiasForces::numReturns()
{
  return(numReturnArguments);
}

bool ModelGeneralisedBiasForces::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelGeneralisedBiasForces\n");
#endif
//   bool returnVal = false;
    if(nlhs!=1)
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs", "one output argument required for computing generalised bias forces.");
    }
//   {
    plhs[0] =  mxCreateNumericMatrix(numDof+6, 1, mxDOUBLE_CLASS, mxREAL);//
//     //plhs[0]=mxCreateDoubleMatrix(numDof+6,numDof+6, mxREAL);
//     //plhs[1]=mxCreateDoubleMatrix(numDof,1, mxREAL);
//   
    h = mxGetPr(plhs[0]);
//    // jointUpperLimit = mxGetPr(plhs[1]);
//     returnVal = true;
//   }
//   
  return(true);
}

bool ModelGeneralisedBiasForces::compute(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute generalised bias forces\n");
#endif
  processArguments(nrhs,prhs);
  /*
bool icubWholeBodyModel::computeGeneralizedBiasForces	(	double * 	q,
const wbi::Frame & 	xBase,
double * 	dq,
double * 	dxB,
double * 	g,
double * 	h */
// )		

  return(true);
}

bool ModelGeneralisedBiasForces::display(int nrhs, const mxArray* prhs[])
{
  mexPrintf("Trying to display generalised bias forces, h:\n");
/*  
  double tempH[modelState->dof()+6];
  robotModel->computeGeneralizedBiasForces(modelState->q(),modelState->baseFrame(),modelState->dq(),modelState->dxb(),g,tempH);
  */
  
//   for (int i = 0;i<modelState->dof()+6;i++)
//   {
//     mexPrintf("%f ",tempH[i]);
//   }
  return(true);
}
bool ModelGeneralisedBiasForces::processArguments(int nrhs, const mxArray* prhs[])
{
  if(nrhs<4)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Atleast two input arguments required for mass-matrix");
  }
  
  if(mxGetM(prhs[1]) != numDof || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != numDof || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 6 || mxGetN(prhs[3]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions");
  }
    
  q = mxGetPr(prhs[1]);
  dq = mxGetPr(prhs[2]);
  dxb = mxGetPr(prhs[3]);
  
#ifdef DEBUG
  mexPrintf("q received \n");

  for(int i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",q[i]);
  }
#endif  

  Eigen::Matrix4d H_w2b;
  int LINK_FOOT_WRF;
  wbi::Frame H_base_wrfLink,xB;
  robotModel->getLinkId ("l_sole", LINK_FOOT_WRF);
  robotModel->computeH(q,wbi::Frame(),LINK_FOOT_WRF, H_base_wrfLink);
  
  H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
  xB.set4x4Matrix (H_w2b.data());
  
  if(h != NULL)
  {
     //robotModel->computeMassMatrix(q,xB,massMatrix);
    robotModel->computeGeneralizedBiasForces(q,xB,dq,dxb,g,h);
    
  }
}

