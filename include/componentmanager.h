/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2014  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef COMPONENTNMAINTAINER_H
#define COMPONENTNMAINTAINER_H

//standard includes
#include <string.h>

// external headers
#include<wbi/iWholeBodyModel.h>
#include <mex.h>

// local includes
#include "modelcomponent.h"
#include "modeljointlimits.h"
#include "modelmassmatrix.h"
// #include "modelstateupdater.h"
#include "modelgeneralisedbiasforces.h"
#include "modelstate.h"
#include "modeldjdq.h"
#include "modeljacobian.h"


namespace mexWBIComponent
{
class ComponentManager
{
public:
  
  static ComponentManager *getInstance();
  bool processFunctionCall(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]);
   ~ComponentManager(void);
   
  int getDofs();
private:
  ComponentManager(void);
  static ComponentManager *componentManager;
  void initialise(void);
  
  ModelState *modelState;
  ModelComponent *currentComponent;
  ModelJointLimits *modelJointLimits;
  ModelMassMatrix *modelMassMatrix;
//   ModelStateUpdater *modelStateUpdater;
  ModelGeneralisedBiasForces *modelGeneralisedBiasForces;
  ModelDjDq *modelDjDq;
  ModelJacobian *modelJacobian;
 
  //ModelStateSetter *stateSetter;
  //ModelGeneralisedForces *modelGeneralisedForces;
  
  int numDof;
  
  static wbi::iWholeBodyModel *robotModel;
  std::map <std::string, ModelComponent*> componentList;
  //std::vector<std::string> componentListNames;
};

}

#endif // WBMODELCOMPONENTNMAINTAINER_H
