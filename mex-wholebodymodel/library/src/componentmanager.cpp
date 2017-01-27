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

// global includes
#include <iostream>
#include <map>

// library includes

// local includes
#include "componentmanager.h"
#include "modelstate.h"
#include "modelcomponent.h"

#include "modelcentroidalmomentum.h"
#include "modelcoriolisbiasforces.h"
#include "modeldjdq.h"
#include "modelforwardkinematics.h"
#include "modelgeneralizedbiasforces.h"
#include "modelgetfloatingbasestate.h"
#include "modelgetstate.h"
#include "modelgravitybiasforces.h"
#include "modelinitialize.h"
#include "modelinitializeurdf.h"
#include "modelinversedynamics.h"
#include "modeljacobian.h"
#include "modeljointlimits.h"
#include "modelmassmatrix.h"
#include "modeltransformationmatrix.h"
#include "modelsetworldframe.h"
#include "modelupdatestate.h"

using namespace mexWBIComponent;

ComponentManager *ComponentManager::componentManager = 0;

ModelState     *ComponentManager::modelState = 0;
ModelComponent *ComponentManager::currentComponent = 0;

ModelCentroidalMomentum    *ComponentManager::modelCentroidalMomentum = 0;
ModelCoriolisBiasForces    *ComponentManager::modelCoriolisBiasForces = 0;
ModelDJdq                  *ComponentManager::modelDJdq = 0;
ModelForwardKinematics     *ComponentManager::modelForwardKinematics = 0;
ModelGeneralizedBiasForces *ComponentManager::modelGeneralizedBiasForces = 0;
ModelGetFloatingBaseState  *ComponentManager::modelGetFloatingBaseState = 0;
ModelGetState              *ComponentManager::modelGetState = 0;
ModelGravityBiasForces     *ComponentManager::modelGravityBiasForces = 0;
ModelInitialize            *ComponentManager::modelInitialize = 0;
ModelInitializeURDF        *ComponentManager::modelInitializeURDF = 0;
ModelInverseDynamics       *ComponentManager::modelInverseDynamics = 0;
ModelJacobian              *ComponentManager::modelJacobian = 0;
ModelJointLimits           *ComponentManager::modelJointLimits = 0;
ModelMassMatrix            *ComponentManager::modelMassMatrix = 0;
ModelSetWorldFrame         *ComponentManager::modelSetWorldFrame = 0;
ModelTransformationMatrix  *ComponentManager::modelTransformationMatrix = 0;
ModelUpdateState           *ComponentManager::modelUpdateState = 0;

std::map<std::string, ModelComponent*> ComponentManager::componentList;

ComponentManager *ComponentManager::getInstance(std::string robotName)
{
  if (componentManager == 0) {
    componentManager = new ComponentManager(robotName);
  }
#ifdef DEBUG
  mexPrintf("ComponentManager initialized.\n");
#endif

  return componentManager;
}

void ComponentManager::deleteInstance()
{
  deleteObject(&componentManager);
}

ComponentManager::ComponentManager(std::string robotName)
{
#ifdef DEBUG
  mexPrintf("ComponentManager constructed.\n");
#endif
  initialize(robotName);
}

void ComponentManager::cleanup()
{
  deleteComponents();
  ModelState::deleteInstance();
#ifdef DEBUG
  mexPrintf("ComponentManager destructed.\n");
#endif
}

void ComponentManager::deleteComponents()
{
  ModelCentroidalMomentum::deleteInstance();
  ModelCoriolisBiasForces::deleteInstance();
  ModelDJdq::deleteInstance();
  ModelForwardKinematics::deleteInstance();
  ModelGeneralizedBiasForces::deleteInstance();
  ModelGetFloatingBaseState::deleteInstance();
  ModelGetState::deleteInstance();
  ModelGravityBiasForces::deleteInstance();
  ModelInitialize::deleteInstance();
  ModelInitializeURDF::deleteInstance();
  ModelInverseDynamics::deleteInstance();
  ModelJacobian::deleteInstance();
  ModelJointLimits::deleteInstance();
  ModelMassMatrix::deleteInstance();
  ModelSetWorldFrame::deleteInstance();
  ModelTransformationMatrix::deleteInstance();
  ModelUpdateState::deleteInstance();
}

ComponentManager::~ComponentManager()
{
#ifdef DEBUG
  mexPrintf("Start cleanup...\n");
#endif
  cleanup();
}

void ComponentManager::initialize(std::string robotName)
{
  modelState = ModelState::getInstance(robotName);
  initComponents();
  initComponentList();
}

void ComponentManager::initComponents()
{
  modelCentroidalMomentum    = ModelCentroidalMomentum::getInstance();
  modelCoriolisBiasForces    = ModelCoriolisBiasForces::getInstance();
  modelDJdq                  = ModelDJdq::getInstance();
  modelForwardKinematics     = ModelForwardKinematics::getInstance();
  modelGeneralizedBiasForces = ModelGeneralizedBiasForces::getInstance();
  modelGetFloatingBaseState  = ModelGetFloatingBaseState::getInstance();
  modelGetState              = ModelGetState::getInstance();
  modelGravityBiasForces     = ModelGravityBiasForces::getInstance();
  modelInitialize            = ModelInitialize::getInstance();
  modelInitializeURDF        = ModelInitializeURDF::getInstance();
  modelInverseDynamics       = ModelInverseDynamics::getInstance();
  modelJacobian              = ModelJacobian::getInstance();
  modelJointLimits           = ModelJointLimits::getInstance();
  modelMassMatrix            = ModelMassMatrix::getInstance();
  modelSetWorldFrame         = ModelSetWorldFrame::getInstance();
  modelTransformationMatrix  = ModelTransformationMatrix::getInstance();
  modelUpdateState           = ModelUpdateState::getInstance();
}

void ComponentManager::initComponentList()
{
  componentList["centroidal-momentum"]   = modelCentroidalMomentum;
  componentList["coriolis-forces"]       = modelCoriolisBiasForces;
  componentList["dJdq"]                  = modelDJdq;
  componentList["forward-kinematics"]    = modelForwardKinematics;
  componentList["generalized-forces"]    = modelGeneralizedBiasForces;
  componentList["get-base-state"]        = modelGetFloatingBaseState;
  componentList["get-state"]             = modelGetState;
  componentList["gravity-forces"]        = modelGravityBiasForces;
  componentList["inverse-dynamics"]      = modelInverseDynamics;
  componentList["jacobian"]              = modelJacobian;
  componentList["joint-limits"]          = modelJointLimits;
  componentList["mass-matrix"]           = modelMassMatrix;
  componentList["model-initialize"]      = modelInitialize;
  componentList["model-initialize-urdf"] = modelInitializeURDF;
  componentList["set-world-frame"]       = modelSetWorldFrame;
  componentList["transformation-matrix"] = modelTransformationMatrix;
  componentList["update-state"]          = modelUpdateState;
}

bool ComponentManager::processFunctionCall(int nlhs, mxArray **plhs, int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to parse mex-arguments...\n");
#endif
  ModelComponent *activeComponent;
  char *strKeyName = mxArrayToString(prhs[0]);
  std::string strInitKey = "model-initialize";
  std::string strInitUrdfKey = "model-initialize-urdf";

  // check if a new robot model will be initialized ...
  if ( !strInitKey.compare(strKeyName) || !strInitUrdfKey.compare(strKeyName) ) {
    if ( !mxIsChar(prhs[1]) ) {
      mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components.");
    }
    std::string newRobotName  = mxArrayToString(prhs[1]);
    std::string currRobotName = modelState->robotName();

    if (currRobotName.compare(newRobotName) != 0) {
      // the model names are different ...
      mexPrintf("\nNew robot model: %s\n", newRobotName.c_str());

      // reset the component list and all components:
      mexPrintf("Resetting all components...\n");
      deleteComponents();
      componentList.clear();

      initComponents();
      initComponentList();
    }
  }

#ifdef DEBUG
  mexPrintf("Searching for the component '%s', of size %d.\n", strKeyName, sizeof(strKeyName));
#endif

  std::map<std::string, ModelComponent*>::iterator comp = componentList.find(strKeyName);
  if (comp == componentList.end()) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Requested component not found. Please request a valid component.");
  }

  activeComponent = comp->second;
  if (nlhs != (int)activeComponent->numReturns()) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of returned parameters in requested component, check the documentations.");
  }

  if ( (nrhs != (int)(1 + activeComponent->numArguments())) &&
       (nrhs != (int)(1 + activeComponent->numAltArguments())) )
  {
    mexPrintf("Requested component uses %d arguments and returns %d items.\n", activeComponent->numArguments(), activeComponent->numReturns());
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of arguments, check the documentations.");
  }
  activeComponent->allocateReturnSpace(nlhs, plhs);

  if (nrhs == (int)(1 + activeComponent->numAltArguments())) {
    activeComponent->computeFast(nrhs, prhs);
    return true;
  }
  // else ...
  activeComponent->compute(nrhs, prhs);
  return true;
}
