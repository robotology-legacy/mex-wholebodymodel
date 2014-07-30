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

#ifndef MODELMASSMATRIX_H
#define MODELMASSMATRIX_H

#include "modelcomponent.h"
#include "wbi/iWholeBodyModel.h"
#include "wbi/wbiUtil.h"


namespace mexWBIComponent{
class ModelMassMatrix : public ModelComponent
{
public:
  
  
  static ModelMassMatrix* getInstance(wbi::iWholeBodyModel *);
  virtual const int numReturns();
  virtual bool display(int, const mxArray *[]);
  virtual bool compute(int, const mxArray *[]);
  
 virtual bool allocateReturnSpace(int, mxArray *[]);

  ~ModelMassMatrix();  
private:
 // ModelJointLimits(int = 0, mxArray * = NULL );
  ModelMassMatrix(wbi::iWholeBodyModel *);
  static ModelMassMatrix* modelMassMatrix; 
  
  bool processArguments(int, const mxArray *[]);
  
  const int numReturnArguments;
  double *massMatrix;
  double *q;
  
  wbi::Frame xBase;

  ModelMassMatrix();
};
}
#endif // MODELMASSMATRIX_H
