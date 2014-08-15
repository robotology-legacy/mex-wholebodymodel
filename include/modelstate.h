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

#include <iostream>
#include <wbi/wbiUtil.h>
#include <wbi/iWholeBodyModel.h>


namespace mexWBIComponent
{
class ModelState
{
public:
   ~ModelState();
   static ModelState * getInstance(std::string = "icub");
   bool setState(double *,double*,double*,wbi::Frame);
   
   double * q();
   double * dq();
   double * dxb();
   wbi::Frame baseFrame();
   int dof();
   void robotModel(std::string);
   wbi::iWholeBodyModel * robotModel(void);
   std::string robotName(void);
   
private:
  ModelState(std::string);
  static ModelState * modelState;
  //double *qS, *dqS, *dxbS;
  double qS[32],dqS[32],dxbS[6];
  wbi::Frame baseS;
  int numDof;
  std::string currentRobotName;
  
  static wbi::iWholeBodyModel *robotWBIModel;
};
}

#endif // MODELSTATE_H
