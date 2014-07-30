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

#include "modelstateupdater.h"

using namespace mexWBIComponent;
  
//   virtual const int numReturns();
//   virtual void display(int, mxArray *[]);
//   virtual void compute(int, mxArray *[]);
//   
//   ~ModelStateSetter();
// private:
//   
//   static ModelStateSetter * modelStateSetter;

ModelStateUpdater *ModelStateUpdater::modelStateUpdater;

ModelStateUpdater::ModelStateUpdater(wbi::iWholeBodyModel *m) : ModelComponent(m), numReturnArguments(0)
{
  mexPrintf("ModelStateUpdater constructed \n");
  
}

ModelStateUpdater * ModelStateUpdater::getInstance(wbi::iWholeBodyModel *m)
{
  if(modelStateSetter == NULL)
  {
    mdelStateUpdater = new ModelStateUpdater(m);
  }
  
  return(modelStateSetter);
}


ModelStateUpdater::~ModelStateUpdater()
{

}

bool ModelStateUpdater::compute(int nrhs, const mxArray *prhs[])
{
  mexPrintf("Trying to compute ModelStateUpdater");
  
  
  return(setState(nrhs,prhs));

}
bool ModelStateUpdater::display(int nrhs, const mxArray* prhs[])
{
  mexPrintf("Trying to display ModelStateUpdater");
  return(setState(nrhs,prhs));
}

bool ModelStateUpdater::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
  bool retVal = false;
  if(nlhs==0)
  {
    plhs = NULL;//mxCreateNumericMatrix(1,1, mxINT8_CLASS, mxREAL);//
    retVal = true;
  }
  //int *retVal =  mxGetPr(plhs[0]);
  //retVal
  return(retVal);

}

bool ModelStateUpdater::setState(int nrhs, const mxArray* prhs[])
{
  bool retVal = false;
  if(nrhs == 3)
  {
    retVal = true;
    
  }
  return(retVal);
}


