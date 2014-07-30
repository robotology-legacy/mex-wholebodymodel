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
#include <iostream>
#include <map>

#include "componentmanager.h"
#include <wbiIcub/icubWholeBodyModel.h>
#include <iostream>
#include <stdio.h>
#include <mex.h>


using namespace mexWBIComponent;

ComponentManager * ComponentManager::componentManager;
wbi::iWholeBodyModel * ComponentManager::robotModel;

ComponentManager* ComponentManager::getInstance()
{
  if(componentManager == NULL) {
    componentManager = new ComponentManager;
  }
#ifdef DEBUG
  mexPrintf("ComponentManager initialised \n");
#endif
  return(componentManager);
}


ComponentManager::ComponentManager()
{
   mexPrintf("ComponentManager constructed \n");
   initialise(); 
   componentList["joint-limits"] = modelJointLimits;
   componentList["mass-matrix"] = modelMassMatrix;
   componentList["generalised-forces"] = modelGeneralisedBiasForces;
   componentList["djdq"] = modelDjDq;
   componentList["jacobian"] = modelJacobian;
  //right-foot-jacobian
  //left-foot-jacobian
  //dj-dq
  //currentComponent = modelJointLimits;//componentList.at(0);
  
}

ComponentManager::~ComponentManager()
{
//   if(modelJointLimits!=NULL)
//   {

//   }
//   if(robotModel!=NULL)
//   {
  delete(robotModel);
//   }
  mexPrintf("icub WholeBodyModel destructed \n");
  delete(modelJointLimits);
  
  mexPrintf("ComponentManager destructed \n");
}

void ComponentManager::initialise()
{
  std::string localName = "wbiTest";
  std::string robotName = "icub";
  robotModel = new wbiIcub::icubWholeBodyModel(localName.c_str(),robotName.c_str(),iCub::iDynTree::iCubTree_version_tag(2,2,true));
  robotModel->addJoints(wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
  mexPrintf("iCub WholeBodyModel initialised \n");
  modelState = ModelState::getInstance(robotModel->getDoFs());
  
  modelJointLimits = ModelJointLimits::getInstance(robotModel);
  modelMassMatrix = ModelMassMatrix::getInstance(robotModel);
//   modelStateUpdater = ModelStateUpdater::getInstance(robotModel);
  modelGeneralisedBiasForces = ModelGeneralisedBiasForces::getInstance(robotModel);
  modelDjDq = ModelDjDq::getInstance(robotModel);
  modelJacobian = ModelJacobian::getInstance(robotModel);
 // activeModelComponent = NULL;
}

int ComponentManager::getDofs()
{
  return(robotModel->getDoFs());
}

bool ComponentManager::processFunctionCall(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  bool returnVal = false;
  ModelComponent *activeComponent;
  char* str;
#ifdef DEBUG
   mexPrintf("Trying to parseMexArguments\n");
#endif
   //   if(nlhs<1)
//   {
//      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","2 output arguments required for joint limits");
//   }
//    if(nrhs>=1)     
//    {
     str=mxArrayToString(prhs[0]);
#ifdef DEBUG
     mexPrintf("Searching for the component '%s', of size  %d\n",str,sizeof(str));
#endif 
     std::map<std::string,ModelComponent*>::iterator search = componentList.find(str);
     if(search == componentList.end())
     {
	mexPrintf("Invalid input, please refer to API doc \n");
     }
     else
     {
       activeComponent = search->second;
//    if(nlhs>=1)
//        {
	 activeComponent->allocateReturnSpace(nlhs,plhs);
	 activeComponent->compute( nrhs, prhs);
	 returnVal = true;
//        }
/*       else
       {
	 activeComponent->display(nrhs,prhs);
	 returnVal = true;
       } */  //modelJointLimits->allocateReturnSpace( nlhs, plhs);
	  //modelJointLimits->display(nrhs, prhs);    
     }
//      mxFree(str);     
//    }   
   return(returnVal);
}
