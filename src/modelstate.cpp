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

#include <iDynTree/ModelIO/URDFDofsImport.h>
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>

// local includes
#include "modelstate.h"

using namespace mexWBIComponent;

ModelState* ModelState::modelState;
wbi::iWholeBodyModel * ModelState::robotWBIModel = NULL;

bool isRobotNameAFile(const std::string & robotName)
{
  // Ugly hack: check the last four char of robotName :
  // if it ends in .xxx (where xxx is is an classical extention)
  // call the robotModelFromURDF , otherwise the usual robotModel
  if( robotName.size() < 4 )
  {
      return false;
  }

  if( robotName[robotName.size()-4] == '.' )
  {
      return true;
  }
}

ModelState::ModelState(std::string robotName) : robot_reference_frame_link_name("l_sole"), fixedLinkComputation(false) //: qS[ndof],dqS[ndof],dxbS[ndof]
{
  yarp::os::Network::init();

  if( isRobotNameAFile(robotName) )
  {
    robotModelFromURDF(robotName);
  }
  else
  {
    robotModel(robotName);
  }

  numDof = robotWBIModel->getDoFs();

  if( fixedLinkComputation )
  {
    this->setReferenceFrameLink(this->robot_reference_frame_link_name);
  }


  qjS = (double*)malloc(sizeof(double) * (numDof));
  qjDotS = (double*)malloc(sizeof(double) * (numDof));

  gS = new double(3);
  gS[0] = 0; gS[1] = 0; gS[2] = -9.81;

}

ModelState::~ModelState()
{

  if(qjDotS != NULL){
      free(qjDotS);
          qjDotS = 0;
  }

  if(qjS != NULL){
      free(qjS);
          qjS = 0;
  }
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

  //  mexPrintf("qj : %2.2f, qjDot : %2.2f \n",qjS[i],qjDotS[i]);
  }
  for(int i=0;i<6;i++)
  {
    vbS[i] = vb_t[i];
    //mexPrintf("vbS ; %2.2f",vbS[i]);
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
void ModelState::setGravity(double *g_temp)
{
  for(int i=0;i<3;i++)
  {
    gS[i] = g_temp[i];
  }
}

double * ModelState::qj()
{
//   return(qS);
  return(&qjS[0]);
}
void ModelState::qj(double *qj_t)
{
  for (int i = 0;i<numDof ; i++)
  {
    qj_t[i] = qjS[i];
  }
}
double * ModelState::qjDot()
{
//   return(dqS);
  return(&qjDotS[0]);
}
void ModelState::qjDot(double *qjDot_t)
{
  for (int i = 0;i<numDof ; i++)
  {
    qjDot_t[i] = qjDotS[i];
  }
}


double * ModelState::vb()
{
//   return(dxbS);
  return(&vbS[0]);
}
void ModelState::vb(double *vb_t)
{
  for(int i=0;i<6;i++)
  {
    vb_t[i] = vbS[i];
  }
}
// wbi::Frame ModelState::rootRotoTrans()
// {
//   return(rootS);
// }

double * ModelState::g(void)
{
  return(&gS[0]);
}
void ModelState::g(double *gT)
{
  for(int i=0;i<3;i++)
  {
    gT[i] = gS[i];
  }
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

  // set YARP_ROBOT_NAME enviromental variable
  // to load the robot-specific configuration files
  // not a clean solution , see discussion in
  // https://github.com/robotology/yarp/issues/593
  // and https://github.com/robotology/mex-wholebodymodel/issues/32
  yarp::os::Network::setEnvironment("YARP_ROBOT_NAME",robotName);

  currentRobotName = robotName;
  std::string localName = "mexWBModel";
  yarp::os::ResourceFinder rf;
  yarp::os::Property yarpWbiOptions;
  //Get wbi options from the canonical file
  rf.setVerbose (true);
  rf.setDefaultConfigFile("yarpWholeBodyInterface.ini");
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
      fprintf(stderr, "[ERR] mex-wholebodymodel: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
  }
  robotWBIModel->addJoints(RobotMainJoints);


  if(!robotWBIModel->init())
  {
    mexPrintf("WBI unable to initialise (usually means unable to connect to chosen robot)\n");
  }

  mexPrintf("mexWholeBodyModel started with robot : %s, Num of Joints : %d \n",robotName.c_str(), robotWBIModel->getDoFs());

}

void  ModelState::robotModelFromURDF(std::string urdfFileName)
{
  if(robotWBIModel != NULL)
  {
    mexPrintf("Deleting older version of robot");
    delete(robotWBIModel);
  }

  std::string localName = "mexWBModel";
  yarp::os::Property yarpWbiOptions;

  //Overwrite the robot parameter that could be present in wbi_conf_file
  currentRobotName = "robotLoadedDirectlyFromURDFfile";
  yarpWbiOptions.put("robot",currentRobotName);
  yarpWbiOptions.put("urdf",urdfFileName.c_str());
  robotWBIModel = new yarpWbi::yarpWholeBodyModel(localName.c_str(), yarpWbiOptions);

  wbi::IDList RobotURDFJoints;

  std::vector<std::string> dofsFromURDF;

  iDynTree::dofsListFromURDF(urdfFileName,dofsFromURDF);

  for(size_t dof = 0; dof < dofsFromURDF.size(); dof++)
  {
    RobotURDFJoints.addID(dofsFromURDF[dof]);
  }

  robotWBIModel->addJoints(RobotURDFJoints);


  if(!robotWBIModel->init())
  {
    mexPrintf("WBI unable to initialise (usually means unable to connect to chosen robot)\n");
  }

  mexPrintf("mexWholeBodyModel started with robot loaded from urdf file : %s, Num of Joints : %d \n",urdfFileName.c_str(), robotWBIModel->getDoFs());

}


std::string ModelState::robotName(void)
{
  return(currentRobotName);
}

void ModelState::setReferenceFrameLink(std::string desLink)
{
  robot_reference_frame_link_name = desLink;

  std::string com("com");
  if(com.compare(desLink)==0)
  {
    robot_reference_frame_link = wbi::iWholeBodyModel::COM_LINK_ID;
  }
  else
  {
    robotWBIModel->getFrameList().idToIndex(desLink.c_str(),robot_reference_frame_link);
  }
  fixedLinkComputation = true;
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
void ModelState::setRootWorldRotoTranslation(wbi::Frame rootWorldFrame )
{
  world_H_rootLink = rootWorldFrame;
  fixedLinkComputation = false;
}

wbi::Frame ModelState::computeRootWorldRotoTranslation(double* q_temp)
{

  if(fixedLinkComputation)
  {
    ModelState::robotWBIModel->computeH(q_temp,wbi::Frame::identity(),robot_reference_frame_link, rootLink_H_ReferenceLink);
    rootLink_H_ReferenceLink.setToInverse().get4x4Matrix (H_w2b.data());
    // H_rootLink
    referenceLink_H_rootLink.set4x4Matrix (H_w2b.data());
    world_H_rootLink = world_H_reference*referenceLink_H_rootLink ;
  }
  return(world_H_rootLink);
}
