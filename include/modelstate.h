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

#ifndef MODELSTATE_H
#define MODELSTATE_H

// global includes
#include <iostream>

// library includes
#include <wbi/wbiUtil.h>
#include <wbi/iWholeBodyModel.h>
#include <Eigen/Core>

//local includes
#include "mexwholebodymodelsettings.h"

#include <mex.h>


namespace mexWBIComponent
{
class ModelState
{
public:
   virtual ~ModelState();
   static ModelState * getInstance(std::string = "icubGazeboSim");

  /**
   * Delete the (static) instance of this component,
   * and set the instance pointer to NULL.
   */
   static void deleteInstance();

   bool setState(double *,double*,double*);

   //void setBaseFrameLink(int);
   void setReferenceFrameLink(std::string);
   void setReferenceToWorldFrameRotoTrans(wbi::Frame);
   void setRootWorldRotoTranslation(wbi::Frame);
   void setGravity(double *g_temp);

   std::string getReferenceFrameLinkName();
   int getReferenceFrameLink(void);
   wbi::Frame getReferenceToWorldFrameRotoTrans(void);
   wbi::Frame getRootWorldRotoTranslation(void);
   wbi::Frame computeRootWorldRotoTranslation(double* q_temp);

   double * qj();
   double * qjDot();
   double * vb();

   double *g();

   void qj(double *);
   void qjDot(double *);
   void vb(double *);

   void g(double *);
   size_t dof();

   /**
    * Load a robot model from a yarpWholeBodyInterface
    * configuration, specifyng the YARP_ROBOT_NAME
    */
   void robotModel(std::string);

   /**
    * Load a robot model from a URDF file path.
    * Using this function, all the dofs of the URDF
    * are automatically added to the interface.
    */
   void robotModelFromURDF(std::string urdfFileName);

   wbi::iWholeBodyModel * robotModel(void);
   std::string robotName(void);

private:
  ModelState(std::string);
  static ModelState * modelState;
  //double *qS, *dqS, *dxbS;
  double vbS[6],*qjS,*qjDotS, gS[3];//;qjS[MEX_WBMODEL_MAX_NUM_JOINTS],qjDotS[MEX_WBMODEL_MAX_NUM_JOINTS],vbS[6];
//   wbi::Frame rootS;
  int robot_reference_frame_link;
  //wbi::Frame H_rootLink_wrWorld;
  //wbi::Frame H_referenceLink_wrWorld;
   //wbi::Frame world_H_root;
   wbi::Frame world_H_reference;

  size_t numDof;
  std::string currentRobotName;
  std::string robot_reference_frame_link_name;

  static wbi::iWholeBodyModel *robotWBIModel;

  bool fixedLinkComputation;

  Eigen::Matrix4d H_w2b;
  //wbi::Frame H_base_wrfLink,
  //wbi::Frame xB;

//   int robot_base_frame_link;
//   wbi::Frame H_rootLink_wrWorld;
//   wbi::Frame H_baseLink_wrWorld;
  wbi::Frame world_H_rootLink;
  wbi::Frame rootLink_H_ReferenceLink;
  wbi::Frame referenceLink_H_rootLink;

};

/**
 * Helper function to delete instances
 *
 * @param pp pointer to the pointer to delete and set to 0
 */
template <class T> void deleteObject(T** pp)
{
    if( *pp != 0 )
    {
        delete *pp;
        *pp = 0;
    }
}

}

#endif // MODELSTATE_H
