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

#ifndef MODELGETFLOATINGBASESTATE_H
#define MODELGETFLOATINGBASESTATE_H

// global includes

// library includes

// local includes
#include "modelcomponent.h"

namespace mexWBIComponent
{
  class ModelGetFloatingBaseState : public ModelComponent
  {
    public:
      static ModelGetFloatingBaseState *getInstance();

      /**
       * Delete the (static) instance of this component,
       * and set the instance pointer to 0.
       */
      static void deleteInstance();

      virtual bool allocateReturnSpace(int nlhs, mxArray **plhs);
      virtual bool compute(int nrhs, const mxArray **prhs);
      virtual bool computeFast(int nrhs, const mxArray **prhs);

      virtual ~ModelGetFloatingBaseState();

    private:
      ModelGetFloatingBaseState();

      static ModelGetFloatingBaseState *modelGetFloatingBaseState;

      // outputs:
      static double *wf_R_b; // orientation of the floating base in matrix form
      static double *wf_p_b; // cartesian position (translation) of the floating base (x_b)
      static double *vb;     // cartesian velocity (dx_b) and the rotational velocity (omega_b) of the floating base orientation
  };

}

#endif // MODELGETFLOATINGBASESTATE_H
