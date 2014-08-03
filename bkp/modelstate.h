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
   static ModelState * getInstance(int);
   bool setState(double *,double*,double*,wbi::Frame);
   
   double * q();
   double * dq();
   double * dxb();
   wbi::Frame baseFrame();
   int dof();
   
   
private:
  ModelState(int);
  static ModelState * modelState;
  double *qS, *dqS, *dxbS;
  wbi::Frame baseS;
  int numDof;
};
}

#endif // MODELSTATE_H
