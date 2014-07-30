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

#ifndef MODELDJDQ_H
#define MODELDJDQ_H

#include "modelcomponent.h"

namespace mexWBIComponent
{
class ModelDjDq : public ModelComponent
{
public:
   static ModelDjDq* getInstance(wbi::iWholeBodyModel *);
  
  virtual const int numReturns();
  virtual bool allocateReturnSpace(int, mxArray*[]);
  virtual bool display(int, const mxArray *[]);
  virtual bool compute(int, const mxArray *[]);
  ~ModelDjDq();
  
private:
  ModelDjDq(wbi::iWholeBodyModel *);
  static ModelDjDq *modelDjDq;
  bool processArguments(int, const mxArray *[]);
  
  double *q;
  double *dq;
  double *dxb;
  double *Djdq;
  char *refLink;
  const int numReturnArguments;
};

}

#endif // MODELDJDQ_H
