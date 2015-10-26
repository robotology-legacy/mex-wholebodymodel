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

// global includes
#include<string.h>
// library includes
#include<yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include<wbi/wbiUtil.h>

//local includes


#include "modelforwardkinematics.h"

using namespace mexWBIComponent;
ModelForwardKinematics * ModelForwardKinematics::modelForwardKinematics;

ModelForwardKinematics::ModelForwardKinematics() : ModelComponent(4,1,1)
{

}

ModelForwardKinematics::~ModelForwardKinematics()
{
#ifdef DEBUG
  mexPrintf("ModelForwardKinematics destructed\n");
#endif
}

void ModelForwardKinematics::deleteInstance()
{
  deleteObject(&modelForwardKinematics);
}

bool ModelForwardKinematics::allocateReturnSpace(int nlhs, mxArray* plhs[])
{

  #ifdef DEBUG
  mexPrintf("Trying to allocateReturnSpace in ModelForwardKinematics\n");
#endif

  bool returnVal = false;

  plhs[0]=mxCreateDoubleMatrix(7,1, mxREAL);
  xT = mxGetPr(plhs[0]);
  returnVal = true;
  return(returnVal);
}
bool ModelForwardKinematics::compute(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Tring to compute in ModelForwardKinematics");
#endif
  processArguments(nrhs,prhs);
  return(true);

}

bool ModelForwardKinematics::processArguments(int nrhs, const mxArray* prhs[])
{
  size_t numDof = modelState->dof();
  if(mxGetM(prhs[1]) != 9 || mxGetN(prhs[1]) != 1 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1  || mxGetM(prhs[3]) != numDof || mxGetN(prhs[3]) != 1 || !mxIsChar(prhs[4]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
  robotModel = modelState->robotModel();
  qj = mxGetPr(prhs[3]);
  refLink = mxArrayToString(prhs[4]);
  double *R_temp,*p_temp;
  R_temp = (double *)mxGetPr(prhs[1]);
  p_temp = (double *)mxGetPr(prhs[2]);

  double tempR[9],tempP[3];
  for(int i = 0;i<9;i++)
  {
    tempR[i] = R_temp[i];
    if(i<3)
     {
       tempP[i] = p_temp[i];
     }
  }
  wbi::Rotation tempRot(tempR);
  wbi::Frame tempFrame(tempRot, tempP);
#ifdef DEBUG
  mexPrintf("qj received \n");

  for(size_t i = 0; i< numDof;i++)
  {
    mexPrintf(" %f",qj[i]);
  }
#endif

  world_H_rootLink = tempFrame;//modelState->computeRootWorldRotoTranslation(qj);

  if(xT != NULL)
  {
    int refLinkID;
    std::string com("com");

    if(com.compare(refLink)==0)
    {
      refLinkID = -1;
    }
    else
    {

      if(!robotModel->getFrameList().idToIndex(refLink, refLinkID))
      {
	mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","forwardKinematics call Link ID does not exist");
      }


    }

    double xTemp[7];
    if(!(robotModel->forwardKinematics(qj,world_H_rootLink,refLinkID,xTemp)))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the forwardKinematics call");
    }


    xT[0] = xTemp[0];
    xT[1] = xTemp[1];
    xT[2] = xTemp[2];

    double axisAngTemp[] = {xTemp[3],xTemp[4],xTemp[5],xTemp[6]},quatTemp[4];

    wbi::Rotation R = wbi::Rotation::axisAngle(axisAngTemp);
#ifdef DEBUG
    std::stringstream ssR;
    ssR<<"AxisAng : ["<<xTemp[3]<<","<<xTemp[4]<<","<<xTemp[5]<<","<<xTemp[6]<<"]\n";
    std::string sR = ssR.str();
    mexPrintf(sR.c_str());
    mexPrintf("Rotation : \n");
    mexPrintf((R.toString()).c_str());
#endif

    R.getQuaternion(quatTemp[1],quatTemp[2],quatTemp[3],quatTemp[0]);
    xT[3] = quatTemp[0];
    xT[4] = quatTemp[1];
    xT[5] = quatTemp[2];
    xT[6] = quatTemp[3];
  }
//   mxFree(q);
  return(true);
}

bool ModelForwardKinematics::computeFast(int nrhs, const mxArray* prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to fastCompute ModelJacobian \n");
#endif

  if(!mxIsChar(prhs[1]))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }
  robotModel = modelState->robotModel();
  qj = modelState->qj();
  //xB = modelState->rootRotoTrans();
  world_H_rootLink = modelState->computeRootWorldRotoTranslation(qj);
 // mexPrintf("BaseFrame : [%2.2f,%2.2f,%2.2f]\n",world_H_rootLink.p[0],world_H_rootLink.p[1],world_H_rootLink.p[2]);
  refLink = mxArrayToString(prhs[1]);
  int refLinkID;

  std::string com("com");

  if(com.compare(refLink)==0)
  {
    refLinkID = -1;
  }
  else
  {
//     robotModel->getLinkId (refLink, refLinkID);
    if(!robotModel->getFrameList().idToIndex(refLink, refLinkID))
    {
     // mexPrintf(sprintf("Requested %s ",refLink));
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","forwardKinematics call Link ID does not exist");
    }
/*
std::string temp2= refLink;

	std::stringstream ss;
	ss<<refLinkID;
   temp2.append(" ID : ");
   temp2.append(ss.str());


   mexPrintf(temp2.c_str()); */
    /*
    mexPrintf("~~~~~ID List: \n");
      std::string temp = robotModel->getFrameList().toString();
      mexPrintf(temp.c_str());
      mexPrintf("~~~~EndID List \n");*/
  }

  double xTemp[7];

  if(!(robotModel->forwardKinematics(qj,world_H_rootLink,refLinkID,xTemp)))
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidInputs","Something failed in the forwardKinematics call");
  }

    xT[0] = xTemp[0];
    xT[1] = xTemp[1];
    xT[2] = xTemp[2];

    double axisAngTemp[] = {xTemp[3],xTemp[4],xTemp[5],xTemp[6]}, quatTemp[4];

    wbi::Rotation R = wbi::Rotation::axisAngle(axisAngTemp);

#ifdef DEBUG
    std::stringstream ssR;
    ssR<<"AxisAng : ["<<xTemp[3]<<","<<xTemp[4]<<","<<xTemp[5]<<","<<xTemp[6]<<"]\n";
    std::string sR = ssR.str();
    mexPrintf(sR.c_str());
    mexPrintf("Rotation : \n");
    mexPrintf((R.toString()).c_str());
#endif

    R.getQuaternion(quatTemp[1],quatTemp[2],quatTemp[3],quatTemp[0]);
    xT[3] = quatTemp[0];
    xT[4] = quatTemp[1];
    xT[5] = quatTemp[2];
    xT[6] = quatTemp[3];
//   wbi::Frame xTF(xTemp);
//     xT[0] = xTF.p[0];
//     xT[1] = xTF.p[1];
//     xT[2] = xTF.p[2];
//     double quatTemp[4];
//     xTF.R.getQuaternion(quatTemp);
//     xT[3] = quatTemp[0];
//     xT[4] = quatTemp[1];
//     xT[5] = quatTemp[2];
//     xT[6] = quatTemp[3];
#ifdef DEBUG
  mexPrintf("ModelJacobian fastComputed\n");
#endif
  return(true);
}

ModelForwardKinematics* ModelForwardKinematics::getInstance()
{
  if(modelForwardKinematics == NULL)
  {
    modelForwardKinematics = new ModelForwardKinematics();
  }
  return(modelForwardKinematics);

}

