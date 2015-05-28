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

#ifndef MODELJACOBIAN_H
#define MODELJACOBIAN_H

#include "modelcomponent.h"
#include "wbi/iWholeBodyModel.h"
#include "wbi/wbiUtil.h"


//local includes
#include "mexwholebodymodelsettings.h"


namespace mexWBIComponent{
class ModelJacobian : public ModelComponent
{
public:
  
  
  static ModelJacobian* getInstance();
//   virtual const int numReturns();
//   virtual bool display(int, const mxArray *[]);
  virtual bool compute(int, const mxArray *[]);  
  virtual bool computeFast(int, const mxArray *[]);
  virtual bool allocateReturnSpace(int, mxArray *[]);

  ~ModelJacobian();  
private:
  ModelJacobian();
  static ModelJacobian* modelJacobian; 
  
  bool processArguments(int, const mxArray *[]);
  
//<<<<<< HEAD
  double *j_colMajor;
  double *j_rowMajor;//[6*(6+MEX_WBMODEL_MAX_NUM_JOINTS)];
//=======
//   double *j;
//   double *temporaryJacobian;
// >>>>>>> 2079d9e9aecaad2016bf292be94bc8c6b2688f1a
  double *qj;
  char * refLink;
  
};
}
#endif // MODELJACOBIAN_H
