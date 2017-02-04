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

#ifndef MODELCORIOLISBIASFORCES_H
#define MODELCORIOLISBIASFORCES_H

// global includes

// library includes

// local includes
#include "modelcomponent.h"

namespace mexWBIComponent
{
  class
ModelCoriolisBiasForces : public ModelComponent
  {
    public:
      static ModelCoriolisBiasForces *getInstance();

      /**
       * Delete the (static) instance of this component,
       * and set the instance pointer to 0.
       */
      static void deleteInstance();

      virtual bool allocateReturnSpace(int nlhs, mxArray **plhs);
      virtual bool compute(int nrhs, const mxArray **prhs);
      virtual bool computeFast(int nrhs, const mxArray **prhs);

      virtual ~ModelCoriolisBiasForces();

    private:
      ModelCoriolisBiasForces();
      static bool processArguments(int nrhs, const mxArray **prhs);

      static ModelCoriolisBiasForces *modelCoriolisBiasForces;

      static double g_0[3]; // 0-gravity vector

      // inputs:
      static double *qj;
      static double *qj_dot;
      static double *vb;
      // output:
      static double *c_qv; // Coriolis and centrifugal forces C(q,dq)
  };

}

#endif // MODELCORIOLISBIASFORCES_H
