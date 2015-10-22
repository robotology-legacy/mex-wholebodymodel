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

#ifndef COMPONENTNMAINTAINER_H
#define COMPONENTNMAINTAINER_H

//standard includes
#include <string.h>

// external headers
#include <mex.h>

// local includes
#include "modelcomponent.h"
#include "modeljointlimits.h"
#include "modelmassmatrix.h"
#include "modelupdatestate.h"
#include "modelgetstate.h"
#include "modelgeneralisedbiasforces.h"
#include "modelstate.h"
#include "modeldjdq.h"
#include "modeljacobian.h"
#include "modelinitialise.h"
#include "modelinitialiseurdf.h"
#include "modelforwardkinematics.h"
#include "modelvisualizetrajectory.h"
#include "modelcentroidalmomentum.h"
#include "modelsetworldframe.h"
#include "modelsetworldlink.h"

namespace mexWBIComponent
{
class ComponentManager
{
public:

  static ComponentManager *getInstance(std::string robotName = "icub");
  bool processFunctionCall(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]);
   ~ComponentManager(void);

  //int getDofs();
private:
  ComponentManager(std::string);
  static ComponentManager *componentManager;
  void initialise(std::string);

  ModelState *modelState;
  ModelComponent *currentComponent;
  ModelJointLimits *modelJointLimits;
  ModelMassMatrix *modelMassMatrix;
  ModelUpdateState *modelUpdateState;
  ModelGetState *modelGetState;
  ModelGeneralisedBiasForces *modelGeneralisedBiasForces;
  ModelDjDq *modelDjDq;
  ModelJacobian *modelJacobian;
  ModelInitialise *modelInitialise;
  ModelInitialiseURDF *modelInitialiseURDF;
  ModelForwardKinematics *modelForwardKinematics;
  ModelVisualizeTrajectory *modelVisualizeTrajectory;
  ModelCentroidalMomentum *modelCentroidalMomentum;
  ModelSetWorldFrame *modelSetWorldFrame;
  ModelSetWorldLink *modelSetWorldLink;

  int numDof;

  //static wbi::iWholeBodyModel *robotModel;
  std::map <std::string, ModelComponent*> componentList;
};

}

#endif // WBMODELCOMPONENTNMAINTAINER_H
