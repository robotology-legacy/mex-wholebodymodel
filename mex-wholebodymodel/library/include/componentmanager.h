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

// standard includes

// external headers
#include <mex.h>

// forward declarations
namespace mexWBIComponent
{
  class ModelCentroidalMomentum;
  class ModelComponent;
  class ModelCoriolisCentrifugalForces;
  class ModelDjDq;
  class ModelForwardKinematics;
  class ModelGeneralisedBiasForces;
  class ModelGetFloatingBaseState;
  class ModelGetState;
  class ModelGravityForces;
  class ModelInitialise;
  class ModelInitialiseURDF;
  class ModelJacobian;
  class ModelJointLimits;
  class ModelMassMatrix;
  class ModelRotoTranslationMatrix;
  class ModelSetWorldFrame;
  class ModelState;
  class ModelUpdateState;
  class ModelVisualizeTrajectory;
}

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
    static void cleanup();

    bool processFunctionCall(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

    ~ComponentManager(void);

  private:
    ComponentManager(std::string);
    void initialise(std::string);

    static ComponentManager *componentManager;

    ModelState *modelState;

    ModelCentroidalMomentum *modelCentroidalMomentum;
    ModelComponent *currentComponent;
    ModelCoriolisCentrifugalForces *modelCoriolisCentrifugalForces;
    ModelDjDq *modelDjDq;
    ModelForwardKinematics *modelForwardKinematics;
    ModelGeneralisedBiasForces *modelGeneralisedBiasForces;
    ModelGetFloatingBaseState *modelGetFloatingBaseState;
    ModelGetState *modelGetState;
    ModelGravityForces *modelGravityForces;
    ModelInitialise *modelInitialise;
    ModelInitialiseURDF *modelInitialiseURDF;
    ModelJacobian *modelJacobian;
    ModelJointLimits *modelJointLimits;
    ModelMassMatrix *modelMassMatrix;
    ModelRotoTranslationMatrix *modelRotoTranslationMatrix;
    ModelSetWorldFrame *modelSetWorldFrame;
    ModelUpdateState *modelUpdateState;
    ModelVisualizeTrajectory *modelVisualizeTrajectory;

    int numDof;

    std::map <std::string, ModelComponent*> componentList;
  };

}

#endif // WBMODELCOMPONENTNMAINTAINER_H
