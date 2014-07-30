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

#include "../include/modeljointlimits.h"
#include <wbiIcub/icubWholeBodyModel.h>
#include <stdio.h>
#include <mex.h>

using namespace mexWBIComponent;
ModelJointLimits* ModelJointLimits::modelJointLimits = NULL;
//const int ModelJointLimits::numReturnArguments = 2;



ModelJointLimits::ModelJointLimits(wbi::iWholeBodyModel * m) : ModelComponent(m),numReturnArguments(2)//(int n, mxArray *p), 
{
  mexPrintf("ModelJointLimits constructed \n");
  ModelComponent::robotModel = m;
  //std::cout<<"in child constructor jointLimits \n";
//   numDof = robotModel->getDoFs();
}

bool ModelJointLimits::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
  mexPrintf("Trying to allocateReturnSpace in ModelJointLimits\n");
  bool returnVal = false;
  if(nlhs!=2)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","2 output arguments required for joint limits");
  }
  
    //p[0] =  mxCreateNumericMatrix(numDof, 1, mxDOUBLE_CLASS, mxREAL);//
    plhs[0]=mxCreateDoubleMatrix(numDof,1, mxREAL);
    plhs[1]=mxCreateDoubleMatrix(numDof,1, mxREAL);
    returnVal = true;  
  
    jointLowerLimit = mxGetPr(plhs[0]);
    jointUpperLimit = mxGetPr(plhs[1]);
    returnVal = true;
  
  
  return(returnVal);
}

ModelJointLimits::~ModelJointLimits()
{
//   std::cout<<"in child desctructor jointLimits \n";
  mexPrintf("ModelJointLimits destructed \n");
  //delete(jointLimits);
}

ModelJointLimits* ModelJointLimits::getInstance(wbi::iWholeBodyModel * m)
{
  if(modelJointLimits == NULL)  
  {
 
    modelJointLimits  = new ModelJointLimits(m);
  }
  return(modelJointLimits);
}

bool ModelJointLimits::display(int nrhs, const mxArray *prhs[])
{
  mexPrintf("Trying to display ModelJointLimits\n");
  double temp1[numDof],temp2[numDof];
  robotModel->getJointLimits(temp1,temp2);
  
  for(int i = 0; i<numDof; i++)
  {
      //std::cout<<" Joint :"<<i<<", MinLimit"<<minJointLimits[i]<<", MaxLimit"<<maxJointLimits[i]<<std::endl;
      mexPrintf(" Joint : %d, MinLimit : %f, MaxLimit : %f \n",i,temp1[i],temp2[i]);
      
      jointLowerLimit[i] = temp1[i];
      jointUpperLimit[i] = temp2[i];
  }
  
  return(true);
}
bool ModelJointLimits::compute(int nrhs, const mxArray *prhs[])
{
   mexPrintf("Trying to compute ModelJointLimits\n");
  double temp1[numDof],temp2[numDof];
  robotModel->getJointLimits(temp1,temp2);
  
  for(int i = 0; i<numDof; i++)
  {
      jointLowerLimit[i] = temp1[i];
      jointUpperLimit[i] = temp2[i];
  }

  return(true);
}

const int ModelJointLimits::numReturns(void)
{
  return(numReturnArguments);
}