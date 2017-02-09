/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: June, 2016 & January, 2017
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

ModelState *ModelState::modelState = 0;
wbi::iWholeBodyModel *ModelState::robotWBIModel = 0;

size_t ModelState::nDof = 0;
char   *ModelState::pstrCurrRobotName = 0;
const char *ModelState::pcstrLocalName = "mexWBModel";

double ModelState::svb[6]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
double ModelState::sg[3]    = {0.0f, 0.0f, 0.0f};
double *ModelState::sqj     = 0;
double *ModelState::sqj_dot = 0;

wbi::Frame ModelState::wf_H_b = wbi::Frame();

bool isRobotNameAFile(const char *pstrRobotName)
{
  std::string fn = pstrRobotName;
  size_t len = fn.size();

  if (len > 4) {
    // extensions with length of 3 or 4 are only allowed:
    size_t pos = fn.rfind('.', len-4);
    if (pos != std::string::npos) {
      // if '.' was found ...
      std::string ext = fn.substr(pos+1, len - pos);
      len = ext.size();
      if ( (len > 2) && (len < 5) ) {
        return true;
      }
    }
  }
  // else, it is not a file name ...
  return false;
}

ModelState::ModelState(const char *pstrRobotName)
{
  if (isRobotNameAFile(pstrRobotName)) {
    robotModelFromURDF(pstrRobotName);
    return;
  }
  // else, load the model from the yarp-WBI directory ...
  robotModel(pstrRobotName);
}

void ModelState::initState()
{
  if ( (sqj == 0) && (sqj_dot == 0) ) {
    sqj     = new double[nDof];
    sqj_dot = new double[nDof];

  #ifdef DEBUG
    mexPrintf("Allocated sqj & sqj_dot.\n");
  #endif
  }

  sg[0] = 0.0f; sg[1] = 0.0f; sg[2] = -9.81f;
}

void ModelState::initRobotModel(const wbi::IDList &jntIDList)
{
  robotWBIModel->addJoints(jntIDList);

  if ( !robotWBIModel->init() ) {
    mexPrintf("WBI unable to initialize (usually means, unable to connect to chosen robot).\n");
  }
  // update the number of DoFs ...
  nDof = robotWBIModel->getDoFs();
}

ModelState::~ModelState()
{
#ifdef DEBUG
  mexPrintf("ModelState destructor called.\n");
#endif

  if (robotWBIModel != 0) {
    delete robotWBIModel;
    robotWBIModel = 0;
  }
#ifdef DEBUG
  mexPrintf("robotWBIModel deleted.\n");
#endif

  if (sqj_dot != 0) {
    delete[] sqj_dot;
    sqj_dot = 0;
  }
#ifdef DEBUG
  mexPrintf("sqj_dot deleted.\n");
#endif

  if (sqj != 0) {
    delete[] sqj;
    sqj = 0;
  }
#ifdef DEBUG
  mexPrintf("sqj deleted.\n");
  mexPrintf("ModelState destructor returning.\n");
#endif
}

ModelState *ModelState::getInstance(const char *pstrRobotName)
{
  if (modelState == 0) {
    modelState = new ModelState(pstrRobotName);
  }
  return modelState;
}

void ModelState::deleteInstance()
{
  deleteObject(&modelState);
  #ifdef DEBUG
    mexPrintf("ModelState deleted.\n");
  #endif
}

bool ModelState::setState(double *qj_t, double *qj_dot_t, double *vb_t)
{
#ifdef DEBUG
  mexPrintf("Trying to update state.\n");
#endif
  memcpy(sqj, qj_t, sizeof(double)*nDof);
  memcpy(sqj_dot, qj_dot_t, sizeof(double)*nDof);
  memcpy(svb, vb_t, sizeof(double)*6);

  return true;
}

void ModelState::setGravity(double *pg)
{
  memcpy(sg, pg, sizeof(double)*3);
}

void ModelState::setBase2WorldTransformation(wbi::Frame frm3d_H)
{
  wf_H_b = frm3d_H;
}

wbi::Frame ModelState::getBase2WorldTransformation()
{
  return wf_H_b;
}

wbi::iWholeBodyModel *ModelState::robotModel()
{
  return robotWBIModel;
}

char *ModelState::robotName()
{
  return pstrCurrRobotName;
}

double *ModelState::qj()
{
  return &sqj[0];
}

void ModelState::qj(double *qj_t)
{
  memcpy(qj_t, sqj, sizeof(double)*nDof);
}

double *ModelState::qj_dot()
{
  return &sqj_dot[0];
}

void ModelState::qj_dot(double *qj_dot_t)
{
  memcpy(qj_dot_t, sqj_dot, sizeof(double)*nDof);
}

double *ModelState::vb()
{
  return &svb[0];
}

void ModelState::vb(double *vb_t)
{
  memcpy(vb_t, svb, sizeof(double)*6);
}

double* ModelState::g()
{
  return &sg[0];
}

void ModelState::g(double *g_t)
{
  memcpy(g_t, sg, sizeof(double)*3);
}

size_t ModelState::dof()
{
  return nDof;
}

void ModelState::robotModel(const char *pstrRobotName)
{
  pstrCurrRobotName = const_cast<char*>(pstrRobotName);

  // set YARP_ROBOT_NAME environmental variable to
  // load the robot-specific configuration files
  // not a clean solution, see discussion in,
  // https://github.com/robotology/yarp/issues/593
  // and https://github.com/robotology/mex-wholebodymodel/issues/32
  yarp::os::Network::setEnvironment("YARP_ROBOT_NAME", pstrCurrRobotName);

  yarp::os::ResourceFinder rf;
  // get wbi options from the canonical file
  rf.setVerbose(true);
  rf.setDefaultConfigFile("yarpWholeBodyInterface.ini");
  rf.configure(0, 0);
  std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");

  yarp::os::Property yarpWbiOptions;
  yarpWbiOptions.fromConfigFile(wbiConfFile);

  // never get the limits from getLimitsFromControlBoard
  // when using mex-WholeBodyModel
  yarpWbiOptions.unput("getLimitsFromControlBoard");

  robotWBIModel = new yarpWbi::yarpWholeBodyModel(pcstrLocalName, yarpWbiOptions);

  wbi::IDList robJntIDList;
  const char *pstrRobJntIDListName = "ROBOT_MEX_WBI_TOOLBOX";
  if ( !yarpWbi::loadIdListFromConfig(pstrRobJntIDListName, yarpWbiOptions, robJntIDList) ) {
    fprintf(stderr, "[ERR] mexWholeBodyModel: Failed to load (WBI) joint-ID list with name \"%s\".\n", pstrRobJntIDListName);
  }

  initRobotModel(robJntIDList);
  initState(); // initialize the state variables ...
  mexPrintf("mexWholeBodyModel started with robot: %s, Num of Joints: %d\n", pstrCurrRobotName, (int)nDof);
}

void ModelState::robotModelFromURDF(const char *pstrURDFileName)
{
  pstrCurrRobotName = const_cast<char*>("robot_loaded_from_URDF_file");

  // overwrite the robot parameter that could be present in wbi_conf_file
  yarp::os::Property yarpWbiOptions;
  yarpWbiOptions.put("robot", pstrCurrRobotName);
  yarpWbiOptions.put("urdf", pstrURDFileName);
  robotWBIModel = new yarpWbi::yarpWholeBodyModel(pcstrLocalName, yarpWbiOptions);

  wbi::IDList urdfJntIDList;
  std::vector<std::string> dofsList;

  iDynTree::dofsListFromURDF(pstrURDFileName, dofsList);

  for (size_t dof=0; dof < dofsList.size(); dof++) {
    urdfJntIDList.addID(dofsList[dof]);
  }

  initRobotModel(urdfJntIDList);
  initState(); // initialize the state variables ...
  mexPrintf("mexWholeBodyModel started with robot loaded from URDF file: %s, Num of Joints: %d\n", pstrURDFileName, (int)nDof);
}
