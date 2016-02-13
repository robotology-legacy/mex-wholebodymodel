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

// forward declarations
namespace mexWBIComponent{
  class ModelState;
  class ModelComponent;
  class ModelJointLimits;
  class ModelUpdateState;
  class ModelGetState;
  class ModelGeneralisedBiasForces;
  class ModelCoriolisCentrifugalForces;
  class ModelGravityForces;
  class ModelDjDq;
  class ModelJacobian;
  class ModelInitialiseURDF;
  class ModelForwardKinematics;
  class ModelVisualizeTrajectory;
  class ModelCentroidalMomentum;
  class ModelMassMatrix;
  class ModelSetWorldFrame;
  class ModelInitialise;
}
//#include "modelsetworldlink.h"

namespace mexWBIComponent
{
  class ComponentManager
  {
  public:

    static ComponentManager *getInstance(std::string robotName = "icub");

    /**
     * Delete the (static) instance of this component,
     * and set the instance pointer to NULL.
     */
    static void deleteInstance();

    bool processFunctionCall(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]);
    void cleanup();
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
    ModelCoriolisCentrifugalForces *modelCoriolisCentrifugalForces;
    ModelGravityForces *modelGravityForces;
    ModelDjDq *modelDjDq;
    ModelJacobian *modelJacobian;
    ModelInitialise *modelInitialise;
    ModelInitialiseURDF *modelInitialiseURDF;
    ModelForwardKinematics *modelForwardKinematics;
    ModelVisualizeTrajectory *modelVisualizeTrajectory;
    ModelCentroidalMomentum *modelCentroidalMomentum;
    ModelSetWorldFrame *modelSetWorldFrame;

    int numDof;

    //static wbi::iWholeBodyModel *robotModel;
    std::map <std::string, ModelComponent*> componentList;
  };

}

#endif // WBMODELCOMPONENTNMAINTAINER_H
