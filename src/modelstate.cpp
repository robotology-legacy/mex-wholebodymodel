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

// library includes
#include <mex.h>
#include <wbiIcub/icubWholeBodyModel.h>
#include <yarp/os/Network.h>

// local includes
#include "modelstate.h"

using namespace mexWBIComponent;

ModelState* ModelState::modelState;
wbi::iWholeBodyModel * ModelState::robotWBIModel = NULL;


ModelState::ModelState(std::string robotName) //: qS[ndof],dqS[ndof],dxbS[ndof]
{
  //this-> 
  yarp::os::Network::init();
//   yarp::os::Network n;
  robotModel(robotName);
  numDof = robotWBIModel->getDoFs();
  
#ifdef DEBUG
  mexPrintf("ModelState constructed with %d \n",ndof); 
#endif
  
 
}

ModelState::~ModelState()
{

 // delete(qS);
 // delete(dqS);
 // delete(dxbS);
  yarp::os::Network::fini();
  delete(robotWBIModel);
  mexPrintf("ModelState destructed\n"); 
}

ModelState *  ModelState::getInstance(std::string robotName)
{
  if(modelState == NULL)
  {
    modelState = new ModelState(robotName);
  }
  return(modelState);
}


bool ModelState::setState(double *qj_t,double *qjDot_t,double *vb_t, wbi::Frame F)
{
#ifdef DEBUG
  mexPrintf("Trying to update state\n");
#endif
  for(int i = 0;i<numDof;i++)
  {
    qjS[i] = qj_t[i];
    qjDotS[i] = qjDot_t[i];
  }
  for(int i=0;i<6;i++)
  {
    vbS[i] = vb_t[i];
  }
  rootS = F;
  return(true);
}
//    setState(double *,double*,double*,wbi:Frame);
//    
//    double * q();
//    double * dq();
//    double * dxb();
//    wbi::Frame baseFrame();

double * ModelState::qj()
{
//   return(qS);
  return(&qjS[0]);
}

double * ModelState::qjDot()
{
//   return(dqS);
  return(&qjDotS[0]);
}
double * ModelState::vb()
{
//   return(dxbS);
  return(&vbS[0]);
}
wbi::Frame ModelState::rootRotoTrans()
{
  return(rootS);
}
int ModelState::dof()
{
  return(numDof);
}

wbi::iWholeBodyModel* ModelState::robotModel(void)
{
  return(robotWBIModel);
}

void  ModelState::robotModel(std::string robotName)
{
  if(robotWBIModel != NULL)
  {
    mexPrintf("Deleting older version of robot");
    delete(robotWBIModel);
  }
  
  currentRobotName = robotName;
     std::string localName = "wbiTest";
  //std::string robotName = robotNameC;
  robotWBIModel = new wbiIcub::icubWholeBodyModel(localName.c_str(),robotName.c_str(),iCub::iDynTree::iCubTree_version_tag(2,2,true));
 // robotWBIModel->addJoints(wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
  robotWBIModel->addJoints(wbiIcub::ICUB_MAIN_JOINTS);
  robotWBIModel->getLimitsFromModel();
  mexPrintf("WholeBodyModel started with robot : %s, Num of Joints : %d \n",robotName.c_str(), robotWBIModel->getDoFs());
  if(!robotWBIModel->init())
  {
    mexPrintf("WholeBodyModel unable to initialise \n");
  }
  
}

std::string ModelState::robotName(void)
{
  return(currentRobotName);
}

//    void setBaseFrameLink(int);
//    void setBaseToWorldFrameRotoTrans(wbi::Frame);
//    
//    int getBaseFrameLink(void);
//    wbi::Frame getBaseToWorldFrameRotoTrans(void);
   
void ModelState::setBaseFrameLink(int bfl)
{
  robot_base_frame_link = bfl;
}
void ModelState::setBaseToWorldFrameRotoTrans(wbi::Frame trans)
{
  H_baseLink_wrWorld = trans;
}
int ModelState::getBaseFrameLink(void)
{
  return(robot_base_frame_link);
} 
wbi::Frame ModelState::getBaseToWorldFrameRotoTrans(void)
{
  return(H_baseLink_wrWorld);
} 
