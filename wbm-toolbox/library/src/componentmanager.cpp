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
ModelInverseDynamics       *ComponentManager::modelInverseDynamics = 0;
ModelJacobian              *ComponentManager::modelJacobian = 0;
ModelJointLimits           *ComponentManager::modelJointLimits = 0;
ModelMassMatrix            *ComponentManager::modelMassMatrix = 0;
ModelSetWorldFrame         *ComponentManager::modelSetWorldFrame = 0;
ModelTransformationMatrix  *ComponentManager::modelTransformationMatrix = 0;
ModelUpdateState           *ComponentManager::modelUpdateState = 0;

const char *ComponentManager::pcstrInitKey     = "model-initialize";
const char *ComponentManager::pcstrInitURDFKey = "model-initialize-urdf";

std::map<const char*, ModelComponent*, cmp_str> ComponentManager::componentList;

ComponentManager *ComponentManager::getInstance(const char *pstrRobotName)
{
  if (componentManager == 0) {
    componentManager = new ComponentManager(pstrRobotName);
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

ComponentManager::ComponentManager(const char *pstrRobotName)
{
#ifdef DEBUG
  mexPrintf("ComponentManager constructed.\n");
#endif
  initialize(pstrRobotName);
}

void ComponentManager::initialize(const char *pstrRobotName)
{
  modelState = ModelState::getInstance(pstrRobotName);
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
  componentList["set-world-frame"]       = modelSetWorldFrame;
  componentList["transformation-matrix"] = modelTransformationMatrix;
  componentList["update-state"]          = modelUpdateState;
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
#ifdef DEBUG
  mexPrintf("Delete all components...\n");
#endif
  ModelCentroidalMomentum::deleteInstance();
  ModelCoriolisBiasForces::deleteInstance();
  ModelDJdq::deleteInstance();
  ModelForwardKinematics::deleteInstance();
  ModelGeneralizedBiasForces::deleteInstance();
  ModelGetFloatingBaseState::deleteInstance();
  ModelGetState::deleteInstance();
  ModelGravityBiasForces::deleteInstance();
  ModelInverseDynamics::deleteInstance();
  ModelJacobian::deleteInstance();
  ModelJointLimits::deleteInstance();
  ModelMassMatrix::deleteInstance();
  ModelSetWorldFrame::deleteInstance();
  ModelTransformationMatrix::deleteInstance();
  ModelUpdateState::deleteInstance();
  // delete (free) the component list ...
  componentList.clear();
}

ComponentManager::~ComponentManager()
{
#ifdef DEBUG
  mexPrintf("Start cleanup...\n");
#endif
  cleanup();
}

bool ComponentManager::processFunctionCall(int nlhs, mxArray **plhs, int nrhs, const mxArray **prhs)
{
#ifdef DEBUG
  mexPrintf("Trying to parse mex-arguments...\n");
#endif
  char *pstrKeyName = mxArrayToString(prhs[0]);
  size_t cmplen = sizeof(pcstrInitKey) + 1;

  if ( (strncmp(pstrKeyName, pcstrInitKey, cmplen) != 0) &&
       (strncmp(pstrKeyName, pcstrInitURDFKey, cmplen) != 0) )
  {
  #ifdef DEBUG
    mexPrintf("Searching for component '%s'.\n", pstrKeyName);
  #endif
    std::map<const char*, ModelComponent*, cmp_str>::iterator comp = componentList.find(pstrKeyName); // search complexity: O(log(n))
    if (comp != componentList.end()) {
      return executeComputation(comp->second, nlhs, plhs, nrhs, prhs);
    }
  }
  else if ( !strncmp(pstrKeyName, pcstrInitKey, cmplen) ||
            !strncmp(pstrKeyName, pcstrInitURDFKey, cmplen) )
  {
    // reinitialize the whole body model either with a new robot model
    // from the yarp-WBI directory, or specified by an URDF file:
    reinitialize(prhs);
    return true;
  }
  // else, the component-key could not be found ...
  mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Requested component not found. Please request a valid component.");
  return false;
}

bool ComponentManager::executeComputation(ModelComponent *pActiveComp, int nlhs, mxArray **plhs, int nrhs, const mxArray **prhs) {
  // check the numbers of the output and input arguments (incl. alternative input arguments) ...
  if (nlhs != (int)pActiveComp->numReturns()) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of returned parameters in requested component, check the documentations.");
  }
  if ( (nrhs != (int)(1 + pActiveComp->numArguments())) &&
       (nrhs != (int)(1 + pActiveComp->numAltArguments())) )
  {
    mexPrintf("Requested component uses %d arguments and returns %d items.\n", pActiveComp->numArguments(), pActiveComp->numReturns());
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of arguments, check the documentations.");
  }
  // allocate the memory spaces for the output arguments in Matlab ...
  pActiveComp->allocateReturnSpace(nlhs, plhs);

  // execute the computation of the specified kinematic/dynamic function:
  if (nrhs == (int)(1 + pActiveComp->numAltArguments())) {
    // perform optimized computation ...
    return pActiveComp->computeFast(nrhs, prhs);
  }
  // else, perform normal computation ...
  return pActiveComp->compute(nrhs, prhs);
}

void ComponentManager::reinitialize(const mxArray **prhs)
{
  if ( !mxIsChar(prhs[1]) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Malformed state dimensions/components.");
  }
  char *pstrNewRobotName  = mxArrayToString(prhs[1]);
  char *pstrCurrRobotName = modelState->robotName();

  if (strcmp(pstrCurrRobotName, pstrNewRobotName) != 0) {
    // the model names are different ...
    mexPrintf("\nNew robot model: %s\n", pstrNewRobotName);

    mexPrintf("Deleting previous version of the robot model.\n\n");
    // reset all components and the component list ...
    cleanup();
    initialize(pstrNewRobotName);

    mexPrintf("Robot name set as: %s\n", modelState->robotName());
  }
}
