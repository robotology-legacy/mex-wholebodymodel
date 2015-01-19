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

#ifndef MODELGETSTATE_H
#define MODELGETSTATE_H

// global includes

// library includes
#include <wbi/iWholeBodyModel.h>

// local includes
#include "mexwholebodymodelsettings.h"
#include "modelcomponent.h"

namespace mexWBIComponent{
  
class ModelGetState : public ModelComponent
{
public:
  static ModelGetState* getInstance();
  
  virtual bool allocateReturnSpace(int, mxArray*[]);
//   virtual bool display(int, const mxArray *[]);
  virtual bool compute(int, const mxArray *[]);
  virtual bool computeFast(int, const mxArray *[]);
  
  
  ~ModelGetState();
private:
  ModelGetState();
  static ModelGetState *modelGetState;
  
  /*wbi::Frame rootRotoTrans;
  wbi::Rotation rootRoto;
  */double rootQuaternion[4];
  
  double* wTb;//[7];
  double* qj;//[MEX_WBMODEL_MAX_NUM_JOINTS];
  double* qjDot;//[MEX_WBMODEL_MAX_NUM_JOINTS];
  double* vb; 
};

}
#endif // MODELGETSTATE_H
