/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: June, 2016
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
#include <iDynTree/ModelIO/URDFDofsImport.h>
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <yarp/os/ResourceFinder.h>

// local includes
#include "modelstate.h"

using namespace mexWBIComponent;

// ModelState *ModelState::modelState = NULL;
boost::scoped_ptr<ModelState> ModelState::modelState(NULL);
wbi::iWholeBodyModel *ModelState::robotWBIModel = NULL;

bool isRobotNameAFile(const std::string &robotName)
{
  // Ugly hack: check the last four char of robotName :
  // if it ends in .xxx or .xxxx (where xxx or xxxx is is an classical extention)
  // call the robotModelFromURDF, otherwise the usual robotModel
  if(robotName.size() < 5)
    return false;

  if( (robotName[robotName.size()-4] == '.') ||
      (robotName[robotName.size()-5] == '.') )
    return true;

  // else ...
  return false;
}

ModelState::ModelState(std::string robotName)
{
  if(isRobotNameAFile(robotName))
    robotModelFromURDF(robotName);
  else
    robotModel(robotName);

  numDof = robotWBIModel->getDoFs();
  qjS    = new double[numDof];
  qjDotS = new double[numDof];

  gS[0] = 0.0f; gS[1] = 0.0f; gS[2] = -9.81f;
}

ModelState::~ModelState()
{
#ifdef DEBUG
  mexPrintf("ModelState destructor called\n");
#endif

  if(robotWBIModel != NULL)
  {
    delete robotWBIModel;
    robotWBIModel = NULL;
    // mexPrintf("deleted_1\n");
  }

  if(qjDotS != NULL) // TO DEBUG: the dynamic allocated arrays qjDotS and qjS cannot be deleted --> access violations!
  {
    // mexPrintf("deleting_2\n");
    delete[] qjDotS;
    qjDotS = NULL;
    // mexPrintf("deleted_2\n");
  }

#ifdef DEBUG
  mexPrintf("free(qjDotS) called\n");
#endif

  if(qjS != NULL)
  {
    // mexPrintf("deleting_3\n");
    delete[] qjS;
    qjS = NULL;
    // mexPrintf("deleted_3\n");
  }

#ifdef DEBUG
  mexPrintf("free(qjS) called\n");
  mexPrintf("ModelState destructor returning\n");
#endif
}

ModelState *ModelState::getInstance(std::string robotName)
{
  // if(modelState == NULL)
  //   modelState = new ModelState(robotName);

  if(modelState == NULL)
    modelState.reset( new ModelState(robotName) );

  // return modelState;
  return modelState.get();
}

/*void ModelState::deleteInstance()
{
  // mexPrintf("ModState::delInst\n");
  // deleteObject(&modelState); // error
} */

bool ModelState::setState(double *qj_t, double *qjDot_t, double *vb_t)
{
#ifdef DEBUG
  mexPrintf("Trying to update state\n");
#endif
  memcpy(qjS, qj_t, sizeof(double)*numDof);
  memcpy(qjDotS, qjDot_t, sizeof(double)*numDof);
  memcpy(vbS, vb_t, sizeof(double)*6);

  return true;
}

void ModelState::setGravity(double *g_temp)
{
  memcpy(gS, g_temp, sizeof(double)*3);
}

double *ModelState::qj()
{
  return &qjS[0];
}

void ModelState::qj(double *qj_t)
{
  memcpy(qj_t, qjS, sizeof(double)*numDof);
}

double *ModelState::qjDot()
{
  return &qjDotS[0];
}

void ModelState::qjDot(double *qjDot_t)
{
  memcpy(qjDot_t, qjDotS, sizeof(double)*numDof);
}

double *ModelState::vb()
{
  return &vbS[0];
}

void ModelState::vb(double *vb_t)
{
  memcpy(vb_t, vbS, sizeof(double)*6);
}

double* ModelState::g(void)
{
  return &gS[0];
}

void ModelState::g(double *gT)
{
  memcpy(gT, gS, sizeof(double)*3);
}

size_t ModelState::dof()
{
  return numDof;
}

wbi::iWholeBodyModel *ModelState::robotModel(void)
{
  return robotWBIModel;
}

void  ModelState::robotModel(std::string robotName)
{
  if(robotWBIModel != NULL)
  {
    mexPrintf("Deleting older version of robot");
    delete robotWBIModel;
  }
  // set YARP_ROBOT_NAME enviromental variable
  // to load the robot-specific configuration files
  // not a clean solution , see discussion in
  // https://github.com/robotology/yarp/issues/593
  // and https://github.com/robotology/mex-wholebodymodel/issues/32
  yarp::os::Network::setEnvironment("YARP_ROBOT_NAME", robotName);

  currentRobotName = robotName;
  std::string localName = "mexWBModel";
  yarp::os::ResourceFinder rf;
  yarp::os::Property yarpWbiOptions;
  //Get wbi options from the canonical file
  rf.setVerbose(true);
  rf.setDefaultConfigFile("yarpWholeBodyInterface.ini");
  rf.configure(0, 0);

  std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");
  yarpWbiOptions.fromConfigFile(wbiConfFile);

  // Never get the limits from getLimitsFromControlBoard
  // When using mex-wholebodymodel
  yarpWbiOptions.unput("getLimitsFromControlBoard");

  robotWBIModel = new yarpWbi::yarpWholeBodyModel(localName.c_str(), yarpWbiOptions);

  wbi::IDList RobotMainJoints;
  std::string RobotMainJointsListName = "ROBOT_MEX_WBI_TOOLBOX";
  if( !yarpWbi::loadIdListFromConfig(RobotMainJointsListName, yarpWbiOptions, RobotMainJoints) )
    fprintf(stderr, "[ERR] mex-wholebodymodel: impossible to load wbiId joint list with name %s\n", RobotMainJointsListName.c_str());

  robotWBIModel->addJoints(RobotMainJoints);

  if( !robotWBIModel->init() )
    mexPrintf("WBI unable to initialise (usually means unable to connect to chosen robot)\n");

  // update the number of dofs
  numDof = robotWBIModel->getDoFs();

  mexPrintf("mexWholeBodyModel started with robot : %s, Num of Joints : %d \n", robotName.c_str(), robotWBIModel->getDoFs());
}

void  ModelState::robotModelFromURDF(std::string urdfFileName)
{
  if(robotWBIModel != NULL)
  {
    mexPrintf("Deleting older version of robot");
    delete robotWBIModel;
  }

  std::string localName = "mexWBModel";
  yarp::os::Property yarpWbiOptions;

  //Overwrite the robot parameter that could be present in wbi_conf_file
  currentRobotName = "robotLoadedDirectlyFromURDFfile";
  yarpWbiOptions.put("robot", currentRobotName);
  yarpWbiOptions.put("urdf", urdfFileName.c_str());
  robotWBIModel = new yarpWbi::yarpWholeBodyModel(localName.c_str(), yarpWbiOptions);

  wbi::IDList RobotURDFJoints;

  std::vector<std::string> dofsFromURDF;

  iDynTree::dofsListFromURDF(urdfFileName, dofsFromURDF);

  for(size_t dof=0; dof < dofsFromURDF.size(); dof++)
    RobotURDFJoints.addID(dofsFromURDF[dof]);

  robotWBIModel->addJoints(RobotURDFJoints);

  if( !robotWBIModel->init() )
    mexPrintf("WBI unable to initialise (usually means unable to connect to chosen robot)\n");

  // update the number of dofs
  numDof = robotWBIModel->getDoFs();

  mexPrintf("mexWholeBodyModel started with robot loaded from urdf file : %s, Num of Joints : %d \n", urdfFileName.c_str(), robotWBIModel->getDoFs());
}

std::string ModelState::robotName(void)
{
  return currentRobotName;
}

wbi::Frame ModelState::getRootWorldRotoTranslation(void)
{
  return world_H_rootLink;
}

void ModelState::setRootWorldRotoTranslation(wbi::Frame rootWorldFrame)
{
  world_H_rootLink = rootWorldFrame;
}
