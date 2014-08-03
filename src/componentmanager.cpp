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
#include <iostream>
#include <stdio.h>
#include <map>
#include <mex.h>

//library includes
#include <wbiIcub/icubWholeBodyModel.h>

//local includes
#include "componentmanager.h"


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
  
}

ComponentManager::~ComponentManager()
{
  delete(robotModel);

#ifdef DEBUG
  mexPrintf("icub WholeBodyModel destructed \n");
  mexPrintf("ComponentManager destructed \n");
#endif

  delete(modelJointLimits); 
}

void ComponentManager::initialise()
{
  std::string localName = "wbiTest";
  std::string robotName = "icub";
  robotModel = new wbiIcub::icubWholeBodyModel(localName.c_str(),robotName.c_str(),iCub::iDynTree::iCubTree_version_tag(2,2,true));
  robotModel->addJoints(wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
  mexPrintf("iCub WholeBodyModel initialised \n");
  
//modelState = ModelState::getInstance(robotModel->getDoFs());
  
  modelJointLimits = ModelJointLimits::getInstance(robotModel);
  modelMassMatrix = ModelMassMatrix::getInstance(robotModel);
//modelStateUpdater = ModelStateUpdater::getInstance(robotModel);
  modelGeneralisedBiasForces = ModelGeneralisedBiasForces::getInstance(robotModel);
  modelDjDq = ModelDjDq::getInstance(robotModel);
  modelJacobian = ModelJacobian::getInstance(robotModel);
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
   
     str=mxArrayToString(prhs[0]);

#ifdef DEBUG
     mexPrintf("Searching for the component '%s', of size  %d\n",str,sizeof(str));
#endif 

     std::map<std::string,ModelComponent*>::iterator search = componentList.find(str);
    if(search == componentList.end())
    {
       mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Requested component not found. Please request a valid component");
     }
     activeComponent = search->second;
     if(nlhs!=activeComponent->numReturns() || nrhs != (1+activeComponent->numArguments()))
    {
     mexPrintf("Requested component uses  uses %d arguments and returns %d items",activeComponent->numArguments(),activeComponent->numReturns());
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Error in number of arguments and returns in requested component, check docs");
    }
	
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
//      }
    return(returnVal);
}