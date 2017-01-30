/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: June, 2016 & January, 2017
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

#include "modelinitialize.h"

using namespace mexWBIComponent;

ModelInitialize *ModelInitialize::modelInitialize = 0;

ModelInitialize::ModelInitialize() : ModelComponent(1, 0, 0)
{
}

ModelInitialize::~ModelInitialize()
{
}

bool ModelInitialize::allocateReturnSpace(int nlhs, mxArray **plhs)
{
  // nothing to do really
  return true;
}

ModelInitialize *ModelInitialize::getInstance()
{
  if (modelInitialize == 0) {
    modelInitialize = new ModelInitialize();
  }
  return modelInitialize;
}

void ModelInitialize::deleteInstance()
{
  deleteObject(&modelInitialize);
}

bool ModelInitialize::compute(int nrhs, const mxArray **prhs)
{
  std::string robotName = mxArrayToString(prhs[1]);

  if (robotName.compare(modelState->robotName()) != 0) {
    modelState->robotModel(robotName);
  }
  mexPrintf("Robot name set as: %s\n", (modelState->robotName()).c_str());
  return true;
}

bool ModelInitialize::computeFast(int nrhs, const mxArray **prhs)
{
  // nothing to do really
  return true;
}
