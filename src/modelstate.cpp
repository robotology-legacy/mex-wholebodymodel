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

#include "modelstate.h"
#include <wbi/iWholeBodyModel.h>
using namespace mexWBIComponent;
#include <mex.h>

ModelState* ModelState::modelState;
/*
double * ModelState::q;
double * ModelState::dq;
double * ModelState::dxb;
int ModelState::numDof;*/

ModelState::ModelState(int ndof)
{
  
  numDof = ndof;
  qS = new double(ndof);
  dqS = new double(ndof);
  dxbS = new double(6);
  
  mexPrintf("ModelState constructed with %d \n",ndof); 
}

ModelState::~ModelState()
{

  delete(qS);
  delete(dqS);
  delete(dxbS);
}

ModelState *  ModelState::getInstance(int nd)
{
  if(modelState == NULL)
  {
    modelState = new ModelState(nd);
  }
  return(modelState);
}


bool ModelState::setState(double *q_t,double *dq_t,double *dxb_t, wbi::Frame F)
{
#ifdef DEBUG
  mexPrintf("Trying to update state\n");
#endif
  for(int i = 0;i<numDof;i++)
  {
    qS[i] = q_t[i];
    dqS[i] = dq_t[i];
  }
  for(int i=0;i<6;i++)
  {
    dxbS[i] = dxb_t[i];
  }
  baseS = F;
  return(true);
}
//    setState(double *,double*,double*,wbi:Frame);
//    
//    double * q();
//    double * dq();
//    double * dxb();
//    wbi::Frame baseFrame();

double * ModelState::q()
{
  return(qS);
}

double * ModelState::dq()
{
  return(dqS);
}
double * ModelState::dxb()
{
  return(dxbS);
}
wbi::Frame ModelState::baseFrame()
{
  return(baseS);
}
int ModelState::dof()
{
  return(numDof);
}
