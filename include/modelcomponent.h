/*
 * Copyright 2014 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef MODELCOMPONENT_H
#define MODELCOMPONENT_H

#include <iostream>
#include <mex.h>
#include <wbi/wbiUtil.h>
//#include <wbi/iWholeBodyModel.h>
#include<wbiIcub/icubWholeBodyModel.h>
//#include "robotwholebodymodel.h"

// #include "modelstate.h"

namespace mexWBIComponent{
  
class ModelComponent{
public:
  
  static ModelComponent* getInstance();
  
  virtual const int numReturns() = 0;
  virtual bool allocateReturnSpace(int, mxArray*[]) = 0;
  virtual bool display(int, const mxArray *[]) = 0;
  virtual bool compute(int, const mxArray *[]) = 0;
  ~ModelComponent();
  
protected:
  
  bool setState(double * , double * , double *, wbi::Frame);
  
  ModelComponent(wbi::iWholeBodyModel*);  
//   int numArguments;
//   mxArray *argumentMexArray;
  /*
  static double *q;
  static double *dq;
  static double *dxb;
  static wbi::Frame xBase;*/
//   
  int numDof;
  wbi::iWholeBodyModel *robotModel;
//   ModelState *modelState;
  
  
  
};


}
#endif // MODELCOMPONENT_H