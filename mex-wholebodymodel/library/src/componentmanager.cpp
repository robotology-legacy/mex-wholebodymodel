/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer, email: martin.neururer@gmail.com
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
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>

//local includes
#include "componentmanager.h"
#include "modelcomponent.h"
#include "modeljointlimits.h"
#include "modelmassmatrix.h"
#include "modelupdatestate.h"
#include "modelgetstate.h"
#include "modelgetfloatingbasestate.h"
#include "modelgeneralisedbiasforces.h"
#include "modelcorioliscentrifugalforces.h"
#include "modelgravityforces.h"
#include "modelstate.h"
#include "modeldjdq.h"
#include "modeljacobian.h"
#include "modelinitialise.h"
#include "modelinitialiseurdf.h"
#include "modelforwardkinematics.h"
#include "modelvisualizetrajectory.h"
#include "modelcentroidalmomentum.h"
#include "modelsetworldframe.h"

using namespace mexWBIComponent;

ComponentManager * ComponentManager::componentManager;

ComponentManager* ComponentManager::getInstance(std::string robotName)
{
  if(componentManager == NULL) {
    componentManager = new ComponentManager(robotName);
  }
#ifdef DEBUG
  mexPrintf("ComponentManager initialised \n");
#endif
  return(componentManager);
}

void ComponentManager::deleteInstance()
{
  deleteObject(&componentManager);
}

ComponentManager::ComponentManager(std::string robotName)
{
#ifdef DEBUG
  mexPrintf("ComponentManager constructed \n");
#endif
  initialise(robotName);
  componentList["joint-limits"] = modelJointLimits;
  componentList["mass-matrix"] = modelMassMatrix;
  componentList["generalised-forces"] = modelGeneralisedBiasForces;
  componentList["coriolis-centrifugal-forces"] = modelCoriolisCentrifugalForces;
  componentList["gravity-forces"] = modelGravityForces;
  componentList["djdq"] = modelDjDq;
  componentList["jacobian"] = modelJacobian;
  componentList["update-state"] = modelUpdateState;
  componentList["get-state"] = modelGetState;
  componentList["get-floating-base-state"] = modelGetFloatingBaseState;
  componentList["model-initialise"] = modelInitialise;
  componentList["model-initialise-urdf"] = modelInitialiseURDF;
  componentList["forward-kinematics"] = modelForwardKinematics;
  componentList["visualize-trajectory"] = modelVisualizeTrajectory;
  componentList["centroidal-momentum"] = modelCentroidalMomentum;
  componentList["set-world-frame"] = modelSetWorldFrame;
}

void ComponentManager::cleanup()
{
#ifdef DEBUG
  mexPrintf("ComponentManager destructed\n");
#endif
  ModelJointLimits::deleteInstance();
  ModelMassMatrix::deleteInstance();
  ModelUpdateState::deleteInstance();
  ModelGetState::deleteInstance();
  ModelGetFloatingBaseState::deleteInstance();
  ModelGeneralisedBiasForces::deleteInstance();
  ModelCoriolisCentrifugalForces::deleteInstance();
  ModelGravityForces::deleteInstance();
  ModelDjDq::deleteInstance();
  ModelJacobian::deleteInstance();
  ModelForwardKinematics::deleteInstance();
  ModelInitialise::deleteInstance();
  ModelInitialiseURDF::deleteInstance();
  ModelVisualizeTrajectory::deleteInstance();
  ModelCentroidalMomentum::deleteInstance();
  ModelSetWorldFrame::deleteInstance();
  ModelState::deleteInstance();
}

ComponentManager::~ComponentManager(void)
{
  cleanup();
}

void ComponentManager::initialise(std::string robotName)
{
  modelState = ModelState::getInstance(robotName);
  modelUpdateState = ModelUpdateState::getInstance();
  modelGetState = ModelGetState::getInstance();
  modelGetFloatingBaseState = ModelGetFloatingBaseState::getInstance();
  modelJointLimits = ModelJointLimits::getInstance();
  modelMassMatrix = ModelMassMatrix::getInstance();
  modelGeneralisedBiasForces = ModelGeneralisedBiasForces::getInstance();
  modelCoriolisCentrifugalForces = ModelCoriolisCentrifugalForces::getInstance();
  modelGravityForces = ModelGravityForces::getInstance();
  modelDjDq = ModelDjDq::getInstance();
  modelJacobian = ModelJacobian::getInstance();
  modelForwardKinematics = ModelForwardKinematics::getInstance();
  modelInitialise = ModelInitialise::getInstance();
  modelInitialiseURDF = ModelInitialiseURDF::getInstance();
  modelVisualizeTrajectory = ModelVisualizeTrajectory::getInstance();
  modelCentroidalMomentum = ModelCentroidalMomentum::getInstance();
  modelSetWorldFrame = ModelSetWorldFrame::getInstance();
}

bool ComponentManager::processFunctionCall(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  bool returnVal = false;
  ModelComponent *activeComponent;
  char *str;

#ifdef DEBUG
  mexPrintf("Trying to parseMexArguments\n");
#endif

  str = mxArrayToString(prhs[0]);

#ifdef DEBUG
  mexPrintf("Searching for the component '%s', of size  %d\n",str,sizeof(str));
#endif

  std::map<std::string,ModelComponent*>::iterator search = componentList.find(str);
  if(search == componentList.end())
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Requested component not found. Please request a valid component");
  }

  activeComponent = search->second;
  if(nlhs != (int)activeComponent->numReturns())
  {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of returned parameters in requested component, check docs");
  }

  if(nrhs != (int)(1+activeComponent->numArguments()) && nrhs != (int)(1+activeComponent->numAltArguments()))
  {
     mexPrintf("Requested component uses  uses %d arguments and returns %d items",activeComponent->numArguments(),activeComponent->numReturns());
     mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of arguments, check docs");
  }
  activeComponent->allocateReturnSpace(nlhs,plhs);

  if(nrhs == (int)(1+activeComponent->numAltArguments()))
  {
    activeComponent->computeFast(nrhs,prhs);
    returnVal = true;
  }
  else
  {
    activeComponent->compute(nrhs, prhs);
    returnVal = true;
  }

  return returnVal;
}
