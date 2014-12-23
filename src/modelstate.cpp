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
// #include <wbiIcub/icubWholeBodyModel.h>
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

// local includes
#include "modelstate.h"

using namespace mexWBIComponent;

ModelState* ModelState::modelState;
wbi::iWholeBodyModel * ModelState::robotWBIModel = NULL;


ModelState::ModelState(std::string robotName) : robot_reference_frame_link_name("l_sole") //: qS[ndof],dqS[ndof],dxbS[ndof]
{
  //this-> 
  yarp::os::Network::init();
//   yarp::os::Network n;
  robotModel(robotName);
  numDof = robotWBIModel->getDoFs();
  this->setReferenceFrameLink(this->robot_reference_frame_link_name);
  
// #ifdef DEBUG
//   mexPrintf("ModelState constructed with %d \n",numDof); 
// #endif
  
 
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


bool ModelState::setState(double *qj_t,double *qjDot_t,double *vb_t)
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
  //rootS = F;
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
// wbi::Frame ModelState::rootRotoTrans()
// {
//   return(rootS);
// }
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
     std::string localName = "mexWBModel";
  //std::string robotName = robotNameC;
  //   robotWBIModel = new wbiIcub::icubWholeBodyModel(localName.c_str(),robotName.c_str(),iCub::iDynTree::iCubTree_version_tag(2,2,true));	
  //   robotWBIModel->addJoints(wbiIcub::ICUB_MAIN_JOINTS);
  yarp::os::ResourceFinder rf;
  yarp::os::Property yarpWbiOptions;
  //Get wbi options from the canonical file
  if( !rf.check("yarp") )
  {
      fprintf(stderr,"[ERR] locomotionControl: impossible to open wholeBodyInterface: wbi_conf_file option missing");
  }
  
  rf.setVerbose (true);
  rf.setDefaultConfigFile ("yarpWholeBodyInterface.ini");
//   rf.setDefaultContext ("icubGazeboSim");
  
  rf.configure(0,0);
  
  std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");
  yarpWbiOptions.fromConfigFile(wbiConfFile);
  //Overwrite the robot parameter that could be present in wbi_conf_file
  yarpWbiOptions.put("robot",robotName);
  robotWBIModel = new yarpWbi::yarpWholeBodyModel(localName.c_str(), yarpWbiOptions);
  
  wbi::IDList RobotMainJoints;
  std::string RobotMainJointsListName = "ROBOT_MEX_WBI_TOOLBOX";
  if( !yarpWbi::loadIdListFromConfig(RobotMainJointsListName,yarpWbiOptions,RobotMainJoints) )
  {
      fprintf(stderr, "[ERR] locomotionControl: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
  }	
  robotWBIModel->addJoints(RobotMainJoints);
  
  
  if(!robotWBIModel->init())
  {
    mexPrintf("WBI unable to initialise (usually means unable to connect to chosen robot)\n");
  }
  
  
  mexPrintf("mexWholeBodyModel started with robot : %s, Num of Joints : %d \n",robotName.c_str(), robotWBIModel->getDoFs());
  
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
   
// void ModelState::setBaseFrameLink(int bfl)
// {
//   robot_base_frame_link = bfl;
// }

void ModelState::setReferenceFrameLink(std::string desLink)
{
  robot_reference_frame_link_name = desLink;
  
  std::string com("com");
  //int robot_base_frame_link;
  //mexPrintf("Old base frame : %d\n",modelState->getBaseFrameLink());
  if(com.compare(desLink)==0)
  {
    robot_reference_frame_link = -1;
  }
  else
  {
//     robotModel->getLinkId (baseLinkName.c_str(), robot_base_frame_link);
      robotWBIModel->getFrameList().idToIndex(desLink.c_str(),robot_reference_frame_link);
      //robotWBIModel->getFrameList().wbiIdToNumericId(desLink.c_str(),robot_base_frame_link);
      //wbiIdToNumericId(baseLinkName.c_str(),robot_base_frame_link);
  }
  
  
  //robotModel->getFrameList().idToIndex(desLink.c_str(),robot_base_frame_link);
  
}

void ModelState::setReferenceToWorldFrameRotoTrans(wbi::Frame trans)
{
  world_H_reference = trans;
}
int ModelState::getReferenceFrameLink(void)
{
  return(robot_reference_frame_link);
} 
std::string ModelState::getReferenceFrameLinkName(void)
{
  return(robot_reference_frame_link_name);
}
wbi::Frame ModelState::getReferenceToWorldFrameRotoTrans(void)
{
  return(world_H_reference);
} 

wbi::Frame ModelState::getRootWorldRotoTranslation(void)
{
  
}


wbi::Frame ModelState::computeRootWorldRotoTranslation(double* q_temp)
{
//  Root_H_referenceLink
//       if(q_temp==NULL)
//       {
// 	q_temp = this->qj;
//       }
      ModelState::robotWBIModel->computeH(q_temp,wbi::Frame::identity(),robot_reference_frame_link, rootLink_H_ReferenceLink);
      rootLink_H_ReferenceLink.setToInverse().get4x4Matrix (H_w2b.data());
     // H_rootLink
      referenceLink_H_rootLink.set4x4Matrix (H_w2b.data());
      world_H_rootLink = world_H_reference*referenceLink_H_rootLink ;
    //  mexPrintf("Current base frame : %d\n",modelState->getBaseFrameLink());
      
      
//   mexPrintf("Inside modelState\nRootWorldRotoTrans\n");
//   
//   mexPrintf("world_H_root\n");
//   mexPrintf((world_H_rootLink.R.toString()).c_str());
//   mexPrintf(" = \n");
//   
//   mexPrintf("world_H_ReferenceLink\n");
//   mexPrintf(( (world_H_reference).R.toString()).c_str());
//   
//   mexPrintf("\n X \n");
//   
//   mexPrintf("referenceLink_H_RootLink\n");
//   mexPrintf((referenceLink_H_rootLink.R.toString()).c_str());
//   
//   mexPrintf("\n\n");
  return(world_H_rootLink);
}
