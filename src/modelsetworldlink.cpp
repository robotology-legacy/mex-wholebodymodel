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

// global includes
#include <string.h>

// library includes
#include <wbi/iWholeBodyModel.h>
#include <wbi/wbiUtil.h>

// local includes
#include "modelsetworldlink.h"

using namespace mexWBIComponent;

ModelSetWorldLink *ModelSetWorldLink::modelSetWorldLink;

ModelSetWorldLink::ModelSetWorldLink(): ModelComponent(4,2,0)
{
  //numDof = robotModel->getDoFs();
#ifdef DEBUG
  mexPrintf("ModelSetWorldLink constructed \n");
#endif

}

ModelSetWorldLink::~ModelSetWorldLink()
{

}

ModelSetWorldLink* ModelSetWorldLink::getInstance()
{
  if(modelSetWorldLink == NULL)
  {
    modelSetWorldLink = new ModelSetWorldLink;
  }
  return(modelSetWorldLink);
}

bool ModelSetWorldLink::allocateReturnSpace(int a, mxArray* m[])
{
  // Nothing to do really
  return(true);
}

bool ModelSetWorldLink::compute(int nrhs, const mxArray* prhs[])
{
  if(!mxIsChar(prhs[1]) || mxGetM(prhs[2]) != 9 || mxGetN(prhs[2]) != 1 || mxGetM(prhs[3]) != 3 || mxGetN(prhs[3]) != 1 || mxGetM(prhs[4]) != 3 || mxGetN(prhs[4]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
  
   double *R_temp,*p_temp,*g_temp;
   
   std::string refLinkName = mxArrayToString(prhs[1]);
   R_temp = (double *)mxGetPr(prhs[2]);
   p_temp = (double *)mxGetPr(prhs[3]);
   g_temp = (double *)mxGetPr(prhs[4]);

   double tempR[9],tempP[3];
   for(int i = 0;i<9;i++)
   {
     tempR[i] = R_temp[i];
      if(i<3)
      {
	tempP[i] = p_temp[i];
      }
   }
    wbi::Rotation tempRot(tempR);
    wbi::Frame tempFrame(tempRot, tempP);
//    yarp::sig::Vector tempVect;
   
//    wbi::serializationFromFrame(tempFrame,tempVect.data());
   
   
   //H_baseLink_wrWorld = temp;
   modelState->setReferenceFrameLink(refLinkName);
   modelState->setReferenceToWorldFrameRotoTrans(tempFrame);
   modelState->setGravity(g_temp);
   
//    changeWorldFrame(refLinkName,tempFrame);
//    mexPrintf("Robot base link changed to %s\n",refLinkName.c_str()); 
}
bool ModelSetWorldLink::computeFast(int nrhs, const mxArray* prhs[])
{
  if(mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }

  double *R_temp,*p_temp;
   R_temp = (double *)mxGetPr(prhs[1]);
   p_temp = (double *)mxGetPr(prhs[2]);
    
   
   double tempR[9],tempP[3];
   for(int i = 0;i<9;i++)
   {
     tempR[i] = R_temp[i];
      if(i<3)
      {
	tempP[i] = p_temp[i];
      }
   }
   wbi::Rotation tempRot(tempR);
   wbi::Frame tempFrame(tempRot, tempP);
   //H_baseLink_wrWorld = temp;
   //H_baseLink_wrWorld = tempFrame;
   modelState->setReferenceToWorldFrameRotoTrans(tempFrame);
//    mexPrintf("Roto translation of world from base frame applied \n"); 
}

 

