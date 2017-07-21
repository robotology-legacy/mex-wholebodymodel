/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * email: naveen.kuppuswamy@iit.it
 * modified by: Martin Neururer; email: martin.neururer@gmail.com; date: June, 2016 & January, 2017
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

// global includes

// library includes

// local includes
#include "modelcomponent.h"

using namespace mexWBIComponent;

ModelState *ModelComponent::modelState = 0;
wbi::iWholeBodyModel *ModelComponent::robotModel = 0;

wbi::Frame ModelComponent::wf_H_b = wbi::Frame();

ModelComponent::ModelComponent(const unsigned int nArgs, const unsigned int nAltArgs, const unsigned int nRets) : numArgs(nArgs), numRets(nRets), numAltArgs(nAltArgs)
{
  modelState = ModelState::getInstance();
  robotModel = modelState->robotModel();

  // initialize/reset the frame transformation H ...
  wf_H_b = wbi::Frame::identity();
}

ModelComponent::~ModelComponent()
{
}

ModelComponent *ModelComponent::getInstance()
{
  return 0;
}

const unsigned int ModelComponent::numReturns()
{
  return numRets;
}

const unsigned int ModelComponent::numArguments()
{
  return numArgs;
}

const unsigned int ModelComponent::numAltArguments()
{
  return numAltArgs;
}

void ModelComponent::reorderMatrixInRowMajor(const double *srcMat, double *destMat, int nRows, int nCols)
{
  // store the values of the matrix-array in row-major order (*):
  // 2D row-major:  offset = i_row*nCols + i_col
  int idx = 0;
  for(int i=0; i < nRows; i++) {
    for(int j=0; j < nCols; j++) {
      *(destMat + idx++) = *(srcMat + (j*nRows + i)); // srcMat is in column-major
      // destMat: idx = (i*nCols + j)
    }
  }
}

void ModelComponent::reorderMatrixInColMajor(const double *srcMat, double *destMat, int nRows, int nCols)
{
  // store the values of the matrix-array in column-major order (*):
  // 2D column-major:  offset = i_col*nRows + i_row
  int idx = 0;
  for(int i=0; i < nCols; i++) {
    for(int j=0; j < nRows; j++) {
      *(destMat + idx++) = *(srcMat + (j*nCols + i)); // srcMat is in row-major
      // destMat: idx = (i*nRows + j)
    }
  }
}

// (*) Further details about the offset calculation for the memory layout of 2D-arrays
//     are available under:
//      [1] Memory layout of multi-dimensional arrays, Eli Bendersky, September 2015,
//            URL: http://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays
//      [2] Row Major and Column Major Address calculations, October 2012,
//            URL: http://www.cbseguy.com/row-column-major-address-calculations-cbse
//      [3] An exhaustive evaluation of row-major, column-major and Morton layouts for large two-dimensional arrays,
//          Thiyagalingam & Beckmann & Kelly, Imperial College, 2003, p. 2,
//            URL: https://www.doc.ic.ac.uk/~phjk/Publications/ExhaustiveMortonUKPEW2003.pdf
//      [4] The Art of Assembly Language Programming, Randall Hyde, 2003, pp. 468-475,
//            URL: http://www.plantation-productions.com/Webster/www.artofasm.com/Linux/HTML/Arraysa2.html
