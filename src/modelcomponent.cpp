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

// global includes
#include <iostream>

// library includes
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>
//#include<wbiIcub/icubWholeBodyModel.h>

// local includes
#include "modelstate.h"
#include "modelcomponent.h"

using namespace mexWBIComponent;

ModelComponent::ModelComponent(const unsigned int args, const unsigned int altArgs, const unsigned int rets) : numArgs(args),numRets(rets),numAltArgs(altArgs)
{
  modelState = ModelState::getInstance();
  robotModel =  modelState->robotModel();
  numDof = robotModel->getDoFs();
  
  int robot_base_frame_link;
  //robotModel->getLinkId ("root", ROBOT_BASE_FRAME_LINK);
  robotModel->getFrameList().idToIndex("l_sole",robot_base_frame_link);
  //robotModel->getFrameList().wbiIdToNumericId("l_sole",robot_base_frame_link);
  //robotModel->getLinkId ("l_sole", robot_base_frame_link);
  modelState->setBaseFrameLink(robot_base_frame_link);
  
}


ModelComponent::~ModelComponent() 
{
  //delete(modelState);
}

ModelComponent* ModelComponent::getInstance()
{
  return(NULL);
}

const unsigned int ModelComponent::numReturns()
{
  return(numRets);
}
const unsigned int ModelComponent::numArguments()
{
  return(numArgs);
}
const unsigned int ModelComponent::numAltArguments()
{
  return(numAltArgs);
}

bool ModelComponent::changeWorldFrame(std::string baseLinkName, wbi::Frame F)
{
  //int refLinkID;
  std::string com("com");
  int robot_base_frame_link;
  //mexPrintf("Old base frame : %d\n",modelState->getBaseFrameLink());
  if(com.compare(baseLinkName)==0)
  {
    robot_base_frame_link = -1;
  }
  else
  {
//     robotModel->getLinkId (baseLinkName.c_str(), robot_base_frame_link);
      robotModel->getFrameList().idToIndex(baseLinkName.c_str(),robot_base_frame_link);
      //wbiIdToNumericId(baseLinkName.c_str(),robot_base_frame_link);
  }
  modelState->setBaseFrameLink(robot_base_frame_link);
  mexPrintf("Base Frame Link ID set to : %d\n",modelState->getBaseFrameLink());
  
  modelState->setBaseToWorldFrameRotoTrans(F);
  return(true);
}

wbi::Frame ModelComponent::computeRootWorldRotoTranslation(double* q_temp)
{
      robotModel->computeH(q_temp,modelState->getBaseToWorldFrameRotoTrans(),modelState->getBaseFrameLink(), H_rootLink_wrWorld);
      H_rootLink_wrWorld.setToInverse().get4x4Matrix (H_w2b.data());
      xB.set4x4Matrix (H_w2b.data());
    //  mexPrintf("Current base frame : %d\n",modelState->getBaseFrameLink());
      return(xB);
}
