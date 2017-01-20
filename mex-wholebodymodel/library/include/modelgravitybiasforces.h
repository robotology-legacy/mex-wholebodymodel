/*
 * Copyright (C) 2016 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Martin Neururer
 * email: martin.neururer@gmail.com, gabriele.nava@iit.it
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

#ifndef MODELGRAVITYBIASFORCES_H
#define MODELGRAVITYBIASFORCES_H

// global includes

// library includes

// local includes
#include "modelcomponent.h"

namespace mexWBIComponent
{
  class ModelGravityBiasForces : public ModelComponent
  {
    public:
      static ModelGravityBiasForces *getInstance();

      /**
       * Delete the (static) instance of this component,
       * and set the instance pointer to 0.
       */
      static void deleteInstance();

      virtual bool allocateReturnSpace(int nlhs, mxArray **plhs);
      virtual bool compute(int nrhs, const mxArray **prhs);
      virtual bool computeFast(int nrhs, const mxArray **prhs);

      virtual ~ModelGravityBiasForces();

    private:
      ModelGravityBiasForces();
      bool processArguments(int nrhs, const mxArray **prhs);

      static ModelGravityBiasForces *modelGravityBiasForces;

      //static double vb_0[6];   // 0-velocity vector
      //static double *qj_dot_0; // zero joint velocities

      // inputs:
      static double *qj;
      static double *g;
      // output:
      static double *g_v;
  };

}

#endif // MODELGRAVITYBIASFORCES_H
