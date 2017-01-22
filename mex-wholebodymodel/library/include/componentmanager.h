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

#ifndef COMPONENTNMAINTAINER_H
#define COMPONENTNMAINTAINER_H

// global includes
#include <mex.h>

// library includes

// local includes

// forward declarations
namespace mexWBIComponent
{
  class ModelState;
  class ModelComponent;
  // ------------------------------------
  // kinematic/dynamic function classes:
  // ------------------------------------
  class ModelCentroidalMomentum;
  class ModelCoriolisBiasForces;
  class ModelDJdq;
  class ModelForwardKinematics;
  class ModelGeneralizedBiasForces;
  class ModelGetFloatingBaseState;
  class ModelGetState;
  class ModelGravityBiasForces;
  class ModelInitialize;
  class ModelInitializeURDF;
  class ModelInverseDynamics;
  class ModelJacobian;
  class ModelJointLimits;
  class ModelMassMatrix;
  class ModelSetWorldFrame;
  class ModelTransformationMatrix;
  class ModelUpdateState;
}

namespace mexWBIComponent
{
  class ComponentManager
  {
    public:
      static ComponentManager *getInstance(std::string robotName = "icub");

      bool processFunctionCall(int nlhs, mxArray **plhs, int nrhs, const mxArray **prhs);

      /**
       * Delete the (static) instance of this component,
       * and set the instance pointer to 0.
       */
      static void deleteInstance();

      ~ComponentManager();

    private:
      ComponentManager(std::string robotName);
      void initialise(std::string robotName);

      static void cleanup();

      static ComponentManager *componentManager;

      static ModelState     *modelState;
      static ModelComponent *currentComponent;

      static ModelCentroidalMomentum    *modelCentroidalMomentum;
      static ModelCoriolisBiasForces    *modelCoriolisBiasForces;
      static ModelDJdq                  *modelDJdq;
      static ModelForwardKinematics     *modelForwardKinematics;
      static ModelGeneralizedBiasForces *modelGeneralizedBiasForces;
      static ModelGetFloatingBaseState  *modelGetFloatingBaseState;
      static ModelGetState              *modelGetState;
      static ModelGravityBiasForces     *modelGravityBiasForces;
      static ModelInitialize            *modelInitialize;
      static ModelInitializeURDF        *modelInitializeURDF;
      static ModelInverseDynamics       *modelInverseDynamics;
      static ModelJacobian              *modelJacobian;
      static ModelJointLimits           *modelJointLimits;
      static ModelMassMatrix            *modelMassMatrix;
      static ModelSetWorldFrame         *modelSetWorldFrame;
      static ModelTransformationMatrix  *modelTransformationMatrix;
      static ModelUpdateState           *modelUpdateState;

      static int numDof;
      static std::map<std::string, ModelComponent*> componentList;
  };

}

#endif // WBMODELCOMPONENTNMAINTAINER_H
