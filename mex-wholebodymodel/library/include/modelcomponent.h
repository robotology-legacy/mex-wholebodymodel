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

#ifndef MODELCOMPONENT_H
#define MODELCOMPONENT_H

#include <iostream>
#include <mex.h>

#include <wbi/iWholeBodyModel.h>

#include <wbi/wbiUtil.h>

#include <Eigen/Core>

#include "modelstate.h"

namespace mexWBIComponent{

class ModelComponent{
public:
  static ModelComponent* getInstance();

  virtual bool allocateReturnSpace(int, mxArray*[]) = 0;
  virtual bool compute(int, const mxArray *[]) = 0;
  virtual bool computeFast(int, const mxArray *[]) = 0;
  //virtual bool display(int, const mxArray *[]) = 0;

  const unsigned int numReturns();
  const unsigned int numArguments();
  const unsigned int numAltArguments();

  virtual ~ModelComponent();

protected:


  ModelComponent(const unsigned int, const unsigned int, const unsigned int);

/* Internal function used to reorder double * matrix
 * elements (since MATLAB is column-major ordered 
 * wbi::Rotation is instead row-major)
 */
  bool reorderMatrixElements(double *sourceMatrix, double (&destinationMatrix)[9]) ;
  

  const unsigned int numArgs;
  const unsigned int numRets;
  const unsigned int numAltArgs;
  wbi::iWholeBodyModel *robotModel;


  Eigen::Matrix4d H_w2b;
  wbi::Frame world_H_rootLink;

  ModelState *modelState;

};


}
#endif // MODELCOMPONENT_H
