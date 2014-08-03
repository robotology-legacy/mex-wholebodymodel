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
static ComponentManager *componentManager;

//=========================================================================================================================
// Entry point function to library
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
  componentManager = ComponentManager::getInstance();
  
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
  componentManager->processFunctionCall(nlhs,plhs,nrhs,prhs);
}


