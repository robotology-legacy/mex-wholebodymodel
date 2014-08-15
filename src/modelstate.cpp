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

// global includes

// library includes
#include <mex.h>
#include <wbiIcub/icubWholeBodyModel.h>
#include <yarp/os/Network.h>

// local includes
#include "modelstate.h"

using namespace mexWBIComponent;

ModelState* ModelState::modelState;
wbi::iWholeBodyModel * ModelState::robotWBIModel = NULL;
/*
double * ModelState::q;
double * ModelState::dq;
double * ModelState::dxb;
int ModelState::numDof;*/

ModelState::ModelState(std::string robotName) //: qS[ndof],dqS[ndof],dxbS[ndof]
{
  //this-> 
  yarp::os::Network::init();
  robotModel(robotName);
  numDof = robotWBIModel->getDoFs();
 //qS = new double(ndof);
 //dqS = new double(ndof);
 //dxbS = new double(6);
  
#ifdef DEBUG
  mexPrintf("ModelState constructed with %d \n",ndof); 
#endif
  
 
}

ModelState::~ModelState()
{

 // delete(qS);
 // delete(dqS);
 // delete(dxbS);
  yarp::os::Network::fini();
  delete(robotWBIModel);
  mexPrintf("ModelState destructed\n"); 
}

ModelState *  ModelState::getInstance(std::string robotName)
{
  if(modelState == NULL)
  {
    modelState = new ModelState(robotName);
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
//   return(qS);
  return(&qS[0]);
}

double * ModelState::dq()
{
//   return(dqS);
  return(&dqS[0]);
}
double * ModelState::dxb()
{
//   return(dxbS);
  return(&dxbS[0]);
}
wbi::Frame ModelState::baseFrame()
{
  return(baseS);
}
int ModelState::dof()
{
  return(numDof);
}

wbi::iWholeBodyModel* ModelState::robotModel(void)
{
  return(robotWBIModel);
}

void  ModelState::robotModel(std::string robotName)
{
  if(robotWBIModel != NULL)
  {
    mexPrintf("Deleting older version of robot");
    delete(robotWBIModel);
  }
  
  currentRobotName = robotName;
     std::string localName = "wbiTest";
  //std::string robotName = robotNameC;
  robotWBIModel = new wbiIcub::icubWholeBodyModel(localName.c_str(),robotName.c_str(),iCub::iDynTree::iCubTree_version_tag(2,2,true));
  robotWBIModel->addJoints(wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
  mexPrintf("WholeBodyModel started with robot : %s \n",robotName.c_str());
  if(!robotWBIModel->init())
  {
    mexPrintf("WholeBodyModel unable to initialise \n");
  }
  
}

std::string ModelState::robotName(void)
{
  return(currentRobotName);
}
