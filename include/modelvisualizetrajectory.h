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

#ifndef MODELVISUALIZETRAJECTORY_H
#define MODELVISUALIZETRAJECTORY_H


// global includes

// library includes
#include<yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>

// project includes
#include "./modelcomponent.h"

namespace mexWBIComponent
{
class ModelVisualizeTrajectory : public ModelComponent
{
public:
  static ModelVisualizeTrajectory* getInstance();

  /**
   * Delete the (static) instance of this component,
   * and set the instance pointer to NULL.
   */
  static void deleteInstance();

  virtual bool allocateReturnSpace(int, mxArray*[]);
//   virtual bool display(int, const mxArray *[]);
  virtual bool compute(int, const mxArray *[]);
  virtual bool computeFast(int, const mxArray *[]);

  ~ModelVisualizeTrajectory();
private:
  ModelVisualizeTrajectory();
  static ModelVisualizeTrajectory *modelVisualizeTrajectory;
//   const int numReturnArguments;

  bool visualizeTrajectory(int, const mxArray *[]);
  bool openPortsToICubGui(void);
  yarp::os::Port leftLeg,rightLeg,leftArm,rightArm,torso,floatingBase;
  yarp::os::Bottle leftLegData,rightLegData,leftArmData,rightArmData,torsoData,floatingBaseData;
  bool portStatus;

  //double qj,t;
};
}
#endif // MODELVISUALIZETRAJECTORY_H
