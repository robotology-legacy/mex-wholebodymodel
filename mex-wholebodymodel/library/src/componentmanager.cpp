/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: June, 2016
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
#include "modelcentroidalmomentum.h"
#include "modelcomponent.h"
#include "modelcorioliscentrifugalforces.h"
#include "modeldjdq.h"
#include "modelforwardkinematics.h"
#include "modelgeneralisedbiasforces.h"
#include "modelgetfloatingbasestate.h"
#include "modelgetstate.h"
#include "modelgravityforces.h"
#include "modelinitialise.h"
#include "modelinitialiseurdf.h"
#include "modelinversedynamics.h"
#include "modeljacobian.h"
#include "modeljointlimits.h"
#include "modelmassmatrix.h"
#include "modelrototranslationmatrix.h"
#include "modelsetworldframe.h"
#include "modelstate.h"
#include "modelupdatestate.h"
#include "modelvisualizetrajectory.h"

using namespace mexWBIComponent;

ComponentManager *ComponentManager::componentManager;

ComponentManager *ComponentManager::getInstance(std::string robotName)
{
  if(componentManager == NULL)
    componentManager = new ComponentManager(robotName);

#ifdef DEBUG
  mexPrintf("ComponentManager initialised\n");
#endif

  return componentManager;
}

void ComponentManager::deleteInstance() // TO DEBUG: the destructor causes sometimes still some access violations!
{
  // mexPrintf("CompManag::deleteInst\n");
  deleteObject(&componentManager); // TO CHECK: sometimes it hangs also there
  // mexPrintf("DONE_4\n");
}

ComponentManager::ComponentManager(std::string robotName)
{
#ifdef DEBUG
  mexPrintf("ComponentManager constructed\n");
#endif
  initialise(robotName);

  //componentList["inverse-dynamics"] = modelInverseDynamics;
  componentList["centroidal-momentum"] = modelCentroidalMomentum;
  componentList["coriolis-centrifugal-forces"] = modelCoriolisCentrifugalForces;
  componentList["djdq"] = modelDjDq;
  componentList["forward-kinematics"] = modelForwardKinematics;
  componentList["generalised-forces"] = modelGeneralisedBiasForces;
  componentList["get-floating-base-state"] = modelGetFloatingBaseState;
  componentList["get-state"] = modelGetState;
  componentList["gravity-forces"] = modelGravityForces;
  componentList["jacobian"] = modelJacobian;
  componentList["joint-limits"] = modelJointLimits;
  componentList["mass-matrix"] = modelMassMatrix;
  componentList["model-initialise"] = modelInitialise;
  componentList["model-initialise-urdf"] = modelInitialiseURDF;
  componentList["rototranslation-matrix"] = modelRotoTranslationMatrix;
  componentList["set-world-frame"] = modelSetWorldFrame;
  componentList["update-state"] = modelUpdateState;
  componentList["visualize-trajectory"] = modelVisualizeTrajectory;
}

void ComponentManager::cleanup()
{
  ModelCentroidalMomentum::deleteInstance();
  ModelCoriolisCentrifugalForces::deleteInstance();
  ModelDjDq::deleteInstance();
  ModelForwardKinematics::deleteInstance();
  ModelGeneralisedBiasForces::deleteInstance();
  ModelGetFloatingBaseState::deleteInstance();
  ModelGetState::deleteInstance();
  ModelGravityForces::deleteInstance();
  ModelInitialise::deleteInstance();
  ModelInitialiseURDF::deleteInstance();
  ModelJacobian::deleteInstance();
  ModelJointLimits::deleteInstance();
  ModelMassMatrix::deleteInstance();
  ModelRotoTranslationMatrix::deleteInstance();
  ModelSetWorldFrame::deleteInstance();
  ModelUpdateState::deleteInstance();
  ModelVisualizeTrajectory::deleteInstance();

  // mexPrintf("DONE_1\n");

  // ModelState::deleteInstance(); // TO DEBUG: at the end of the ctest it causes always a segmentation violation!

  // mexPrintf("DONE_2\n");

#ifdef DEBUG
  mexPrintf("ComponentManager destructed\n");
#endif
}

ComponentManager::~ComponentManager()
{
  // mexPrintf("start cleanup\n");
  cleanup();
  // deleteObject(&modelState); // error

  // mexPrintf("DONE_3\n");
}

void ComponentManager::initialise(std::string robotName)
{
  modelState = ModelState::getInstance(robotName);

  modelCentroidalMomentum = ModelCentroidalMomentum::getInstance();
  modelCoriolisCentrifugalForces = ModelCoriolisCentrifugalForces::getInstance();
  modelDjDq = ModelDjDq::getInstance();
  modelForwardKinematics = ModelForwardKinematics::getInstance();
  modelGeneralisedBiasForces = ModelGeneralisedBiasForces::getInstance();
  modelGetFloatingBaseState = ModelGetFloatingBaseState::getInstance();
  modelGetState = ModelGetState::getInstance();
  modelGravityForces = ModelGravityForces::getInstance();
  modelInitialise = ModelInitialise::getInstance();
  modelInitialiseURDF = ModelInitialiseURDF::getInstance();
  modelJacobian = ModelJacobian::getInstance();
  modelJointLimits = ModelJointLimits::getInstance();
  modelMassMatrix = ModelMassMatrix::getInstance();
  modelRotoTranslationMatrix = ModelRotoTranslationMatrix::getInstance();
  modelSetWorldFrame = ModelSetWorldFrame::getInstance();
  modelUpdateState = ModelUpdateState::getInstance();
  modelVisualizeTrajectory = ModelVisualizeTrajectory::getInstance();
}

bool ComponentManager::processFunctionCall(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to parseMexArguments\n");
#endif
  ModelComponent *activeComponent;
  char *str = mxArrayToString(prhs[0]);

#ifdef DEBUG
  mexPrintf("Searching for the component '%s', of size  %d\n", str, sizeof(str));
#endif

  std::map<std::string, ModelComponent*>::iterator search = componentList.find(str);
  if(search == componentList.end())
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Requested component not found. Please request a valid component");

  activeComponent = search->second;
  if(nlhs != (int)activeComponent->numReturns())
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of returned parameters in requested component, check docs");

  if( (nrhs != (int)(1 + activeComponent->numArguments())) &&
      (nrhs != (int)(1 + activeComponent->numAltArguments())) )
  {
    mexPrintf("Requested component uses  uses %d arguments and returns %d items", activeComponent->numArguments(), activeComponent->numReturns());
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidInputs", "Error in number of arguments, check docs");
  }
  activeComponent->allocateReturnSpace(nlhs, plhs);

  if(nrhs == (int)(1 + activeComponent->numAltArguments()))
  {
    activeComponent->computeFast(nrhs, prhs);
    return true;
  }
  // else ...
  activeComponent->compute(nrhs, prhs);
  return true;
}
