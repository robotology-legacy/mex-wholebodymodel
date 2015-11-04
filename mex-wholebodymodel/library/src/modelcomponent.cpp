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

// local includes
#include "modelstate.h"
#include "modelcomponent.h"

using namespace mexWBIComponent;

ModelComponent::ModelComponent(const unsigned int args, const unsigned int altArgs, const unsigned int rets) : numArgs(args),numRets(rets),numAltArgs(altArgs)
{
  modelState = ModelState::getInstance();
  robotModel =  modelState->robotModel();

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

