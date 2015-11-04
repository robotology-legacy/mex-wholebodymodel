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


// global includes
#include <iostream>
#include <stdio.h>
#include <mex.h>

// Local includes
#include <modelcomponent.h>
#include <componentmanager.h>

// Namespaces
using namespace mexWBIComponent;

//Global variables

/**
 * just a local copy of the pointer that you get
 * by ComponentManager::getInstance()
 *
 * To properly destroy, call ComponentManager::deleteInstance()
 * and then set this pointer to 0.
 */
static ComponentManager *componentManagerLocalPointerCopy = 0;

// Cleanup function to call when matlab exits or mex clears
void MEXWBM_Matlab_ExitFcn(void)
{
  ComponentManager::deleteInstance();
  componentManagerLocalPointerCopy = 0;
}

//=========================================================================================================================
// Entry point function to library
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
  if(componentManagerLocalPointerCopy==0)
  {
    // Initialisation of the component, i.e first call after a 'close all' or matlab start
    if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Initialisation has not been performed correctly.");
    }

    // Check to be sure input is of type char
    if (!(mxIsChar(prhs[0])))
    {
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component.\n.");
    }

    if(nrhs ==2)
    {
      if (!(mxIsChar(prhs[1])))
      {
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component and a robot name.\n.");
      }

      componentManagerLocalPointerCopy = ComponentManager::getInstance(mxArrayToString(prhs[1]));
    }
    else
    {
      componentManagerLocalPointerCopy = ComponentManager::getInstance();
    }

    // Register function to call on Matlab close / mex clear
    // to proper deallocate all the memory
    mexAtExit(MEXWBM_Matlab_ExitFcn);
  }
  else
  {
    componentManagerLocalPointerCopy = ComponentManager::getInstance();
  }

#ifdef DEBUG
  mexPrintf("starting to process function\n");
#endif

  if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Required Component must be named.");
    }

// Check to be sure input is of type char
    if (!(mxIsChar(prhs[0]))){
        mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Input must be of type string.\n.");
    }
  componentManagerLocalPointerCopy->processFunctionCall(nlhs,plhs,nrhs,prhs);
}


