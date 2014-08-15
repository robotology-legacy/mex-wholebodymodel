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

ModelInitialise::ModelInitialise() : ModelComponent(1,0,0)
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

ModelInitialise* ModelInitialise::getInstance()
{
  if(modelInitialise == NULL)
  {
    modelInitialise = new ModelInitialise();
  }
  return(modelInitialise);
}

bool ModelInitialise::compute(int nrhs, const mxArray* prhs[])
{
   
  if(!mxIsChar(prhs[1]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
  
  std::string rName = mxArrayToString(prhs[1]);
  
  mexPrintf("Existing Robot name %s and ModelStates robot name %s \n",rName.c_str(),(modelState->robotName()).c_str());
  
  if(rName.compare(modelState->robotName())!=0)
  {
    modelState->robotModel(rName);
  }
  return(true);
  
}

bool ModelInitialise::computeFast(int nrhs, const mxArray* prhs[])
{
  // nothing to do really
  return(true);
}
