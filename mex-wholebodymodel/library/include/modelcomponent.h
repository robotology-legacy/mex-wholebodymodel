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

#ifndef MODELCOMPONENT_H
#define MODELCOMPONENT_H

// global includes

// library includes

// local includes
#include "modelstate.h"

namespace mexWBIComponent
{
  class ModelComponent {
    public:
      static ModelComponent *getInstance();

      virtual bool allocateReturnSpace(int nlhs, mxArray **plhs) = 0;
      virtual bool compute(int nrhs, const mxArray **prhs) = 0;
      virtual bool computeFast(int nrhs, const mxArray **prhs) = 0;

      const unsigned int numReturns();
      const unsigned int numArguments();
      const unsigned int numAltArguments();

      virtual ~ModelComponent();

    protected:
      ModelComponent(const unsigned int nArgs, const unsigned int nAltArgs, const unsigned int nRets);

      /* Internal function used to reorder double* matrix
       * elements (since MATLAB is column-major ordered
       * wbi::Rotation is instead row-major)
       */
      static void reorderMatrixInRowMajor(const double *srcMat, double *destMat, int nRows = 3, int nCols = 3);
      static void reorderMatrixInColMajor(const double *srcMat, double *destMat, int nRows = 3, int nCols = 3);

      static ModelState *modelState;
      static wbi::iWholeBodyModel *robotModel;

      // frame transformation (from base to world frame):
      static wbi::Frame wf_H_b;

      const unsigned int numArgs;
      const unsigned int numRets;
      const unsigned int numAltArgs;
  };

}

#endif // MODELCOMPONENT_H

