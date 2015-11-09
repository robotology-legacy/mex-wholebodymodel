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

   void setRootWorldRotoTranslation(wbi::Frame);
   void setGravity(double *g_temp);

   wbi::Frame getRootWorldRotoTranslation(void);

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
  double vbS[6],*qjS,*qjDotS, gS[3];

  size_t numDof;
  std::string currentRobotName;

  static wbi::iWholeBodyModel *robotWBIModel;

  Eigen::Matrix4d H_w2b;

  wbi::Frame world_H_rootLink;

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
