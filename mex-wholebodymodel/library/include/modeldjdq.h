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

#ifndef MODELDJDQ_H
#define MODELDJDQ_H

// global includes

// library includes

// local includes
#include "modelcomponent.h"

namespace mexWBIComponent
{
  class ModelDjDq : public ModelComponent
  {
    public:
      static ModelDjDq *getInstance(void);

      /**
       * Delete the (static) instance of this component,
       * and set the instance pointer to NULL.
       */
      static void deleteInstance();

      virtual bool allocateReturnSpace(int, mxArray*[]);
      virtual bool compute(int, const mxArray *[]);
      virtual bool computeFast(int, const mxArray *[]);

      virtual ~ModelDjDq();

    private:
      ModelDjDq(void);
      bool processArguments(int, const mxArray *[]);

      static ModelDjDq *modelDjDq;

      double *qj;
      double *qjDot;
      double *vb;
      char   *refLink;
      double *Djdq;
  };

}

#endif // MODELDJDQ_H
