/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 *  Authors: Naveen Kuppuswamy
 *  email: naveen.kuppuswamy@iit.it
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

#include "modelinitialise.h"

using namespace mexWBIComponent;

ModelInitialise * ModelInitialise::modelInitialise;

ModelInitialise::ModelInitialise(wbi::iWholeBodyModel*m) : ModelComponent(m,0,1,0)
{

}

ModelInitialise::~ModelInitialise()
{

}

bool ModelInitialise::allocateReturnSpace(int, mxArray*[])
{
  // nothing to do really
  return(true);
}

ModelInitialise* ModelInitialise::getInstance(wbi::iWholeBodyModel*m)
{
  if(modelInitialise == NULL)
  {
    modelInitialise = new ModelInitialise(m);
  }
  return(modelInitialise);
}

bool ModelInitialise::compute(int nrhs, const mxArray* prhs[])
{

  // nothing to do really
  return(true);
  
}

bool ModelInitialise::computeFast(int nrhs, const mxArray* prhs[])
{
  // nothing to do really
  return(true);
}
