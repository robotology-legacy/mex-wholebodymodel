/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: January, 2017
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

// global includes

// library includes

// local includes
#include "modelcomponent.h"
#include "componentmanager.h"

// namespaces
using namespace mexWBIComponent;

// global variables

/**
 * Just a local copy of the pointer that you get
 * by ComponentManager::getInstance()
 *
 * To properly destroy, call ComponentManager::deleteInstance()
 * and then set this pointer to 0.
 */
static ComponentManager *pCompManager = 0;

// Cleanup function to call when matlab exits or mex clears
void mexWbmExit(void)
{
#ifdef DEBUG
  mexPrintf("Try to delete ComponentManager...\n");
#endif
  ComponentManager::deleteInstance();
  pCompManager = 0;
}

//=========================================================================================================================
// Entry point function to library
void mexFunction(int nlhs, mxArray **plhs, int nrhs, const mxArray **prhs)
{
  if (pCompManager == 0) {
    // initialization of the component, i.e first call after a 'close all' or matlab start
    if (nrhs < 1) {
      mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Initialization has not been performed correctly.");
    }
    // check to be sure input is of type char
    if ( !(mxIsChar(prhs[0])) ) {
      mexErrMsgIdAndTxt("MATLAB:mexatexit:inputNotString", "Initialization must include component.");
    }

    if (nrhs == 2) {
      if ( !(mxIsChar(prhs[1])) ) {
        mexErrMsgIdAndTxt("MATLAB:mexatexit:inputNotString", "Initialization must include component and a robot name.");
      }
      pCompManager = ComponentManager::getInstance(mxArrayToString(prhs[1]));
    }
    else {
      pCompManager = ComponentManager::getInstance();
    }

    // register function to call on Matlab close / mex clear
    // to proper deallocate all the memory
    mexAtExit(mexWbmExit);
  }
  else {
    pCompManager = ComponentManager::getInstance();
  }

#ifdef DEBUG
  mexPrintf("Start to process function...\n");
#endif

  if (nrhs < 1) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:invalidNumInputs", "Required component must be named.");
  }
  // check to be sure input is of type char
  if ( !(mxIsChar(prhs[0])) ) {
    mexErrMsgIdAndTxt("MATLAB:mexatexit:inputNotString", "Input must be of type string.");
  }
  pCompManager->processFunctionCall(nlhs, plhs, nrhs, prhs);
}
