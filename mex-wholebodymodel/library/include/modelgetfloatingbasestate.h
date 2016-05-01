/*
 * Copyright (C) 2016 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 *  Authors: Martin Neururer
 *  email: martin.neururer@gmail.com, gabriele.nava@iit.it
 *
 *  The development of this software was supported by the FP7 EU projects
 *  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 *  http://www.codyco.eu
 *
 *  Permission is granted to copy, distribute, and/or modify this program
 *  under the terms of the GNU General Public License, version 2 or any
 *  later version published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details
 *
 *
 */

#ifndef MODELGETFLOATINGBASESTATE_H
#define MODELGETFLOATINGBASESTATE_H

// global includes

// library includes
#include <wbi/iWholeBodyModel.h>

// local includes
#include "mexwholebodymodelsettings.h"
#include "modelcomponent.h"

namespace mexWBIComponent
{
  class ModelGetFloatingBaseState : public ModelComponent
  {
    public:
      static ModelGetFloatingBaseState* getInstance();

      /**
       * Delete the (static) instance of this component,
       * and set the instance pointer to NULL.
       */
      static void deleteInstance();

      virtual bool allocateReturnSpace(int, mxArray**);
      virtual bool compute(int, const mxArray**);
      virtual bool computeFast(int, const mxArray**);

      //virtual bool display(int, const mxArray *[]);
      virtual ~ModelGetFloatingBaseState();
    private:
      ModelGetFloatingBaseState();
      static ModelGetFloatingBaseState *modelGetFloatingBaseState;

      double *w_R_b; // orientation of the floating base in axis-angle representation (w_R_b)
      double *w_p_b; // cartesian position (translation) of the floating base (x_b)
      double *vb;    // cartesian velocity (dx_b) and the rotational velocity of the floating base orientation (omega_b)
  };

}

#endif // MODELGETFLOATINGBASESTATE_H
