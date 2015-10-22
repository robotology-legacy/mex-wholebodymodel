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

//global includes

// library includes
#include<yarp/os/Time.h>
#include<wbi/wbiUtil.h>

// local includes
#include "modelvisualizetrajectory.h"


using namespace mexWBIComponent;

ModelVisualizeTrajectory *ModelVisualizeTrajectory::modelVisualizeTrajectory;

//   bool setState(int, const mxArray *[]);


ModelVisualizeTrajectory::ModelVisualizeTrajectory() : ModelComponent(3,3,0)
{
  portStatus = false;
}

ModelVisualizeTrajectory::~ModelVisualizeTrajectory()
{

}


ModelVisualizeTrajectory* ModelVisualizeTrajectory::getInstance()
{
  if(modelVisualizeTrajectory == NULL)
  {
    modelVisualizeTrajectory = new ModelVisualizeTrajectory;
  }
  return(modelVisualizeTrajectory);
}

void ModelVisualizeTrajectory::deleteInstance()
{
  deleteObject(&modelVisualizeTrajectory);
}

bool ModelVisualizeTrajectory::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
  // nothing to do really
  return(true);
}

bool ModelVisualizeTrajectory::compute(int nrhs, const mxArray* prhs[])
{
  visualizeTrajectory(nrhs,prhs);
  return(true);
}

bool ModelVisualizeTrajectory::computeFast(int nrhs, const mxArray* prhs[])
{
  visualizeTrajectory(nrhs,prhs);
  return(true);
}
bool ModelVisualizeTrajectory::visualizeTrajectory(int nrhs, const mxArray* prhs[])
{
//   if(!portStatus)
//   {
    openPortsToICubGui();
//   }

  if(mxGetN(prhs[1]) != 1 || mxGetM(prhs[1]) != mxGetM(prhs[2]) || mxGetN(prhs[2]) != modelState->dof() || mxGetM(prhs[1]) != mxGetM(prhs[3]) || mxGetN(prhs[3]) != 7)
  {
     mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions/components");
  }

  int ti=0,numTSteps = mxGetM(prhs[1]),i;
  double *t;
  t= mxGetPr(prhs[1]);
  double *qj;//[numTSteps][modelState->dof()];
  qj= mxGetPr(prhs[2]);
  double * tXb;
  tXb = mxGetPr(prhs[3]);
  double quatVal[4];
  double roll,pitch,yaw;
  double iniTime, finTime,delayTime;

  wbi::Rotation baseRot;

  mexPrintf("Starting to send data to iCubGui\n");
  double totDelayTime = 0.0;
  while(ti < numTSteps)
  {
    iniTime = yarp::os::Time::now();

    //floatingBase

    floatingBaseData.clear();
    for(i=3;i<8;i++)
    {
      quatVal[i-3] = tXb[ti+i*numTSteps];
    }

    baseRot = wbi::Rotation::quaternion(quatVal[0],quatVal[1],quatVal[2],quatVal[3]);

    baseRot.getEulerZYX(roll,pitch,yaw);

    // 3orientations and 3 positions
    floatingBaseData.addDouble(yaw);
    floatingBaseData.addDouble(pitch);
    floatingBaseData.addDouble(roll);
    floatingBaseData.addDouble(1000*tXb[ti+0*numTSteps]);
    floatingBaseData.addDouble(1000*tXb[ti+1*numTSteps]);
    floatingBaseData.addDouble(1000*tXb[ti+2*numTSteps]);

    //torso
    torsoData.clear();
    for (i = 0; i<3;i++)
    {
//       torsoData.addDouble(qj[ti*modelState->dof()+i]);
      torsoData.addDouble(qj[ti+i*numTSteps] );//[ti*modelState->dof()+i]);

    }

//     mexPrintf("Time : %f, Torso : %f, %f, %f \n", t[ti],qj[ti*modelState->dof()+0],qj[ti*modelState->dof()+1],qj[ti*modelState->dof()+2]);
//     mexPrintf("Time : %f, Torso : %f, %f, %f \n", t[ti],qj[ti+0*numTSteps],qj[ti+1*numTSteps],qj[ti+2*numTSteps]);

    //left_arm
    leftArmData.clear();
    for(i =3;i<7;i++)
    {
     // leftArmData.addDouble(qj[ti*modelState->dof()+i]);
      leftArmData.addDouble(qj[ti+i*numTSteps] );
    }


    //right_arm
    rightArmData.clear();
    for(i =8;i<12;i++)
    {
//       rightArmData.addDouble(qj[ti*modelState->dof()+i]);
      rightArmData.addDouble(qj[ti+i*numTSteps] );
    }

    //padding to make the rest of arms render (minor issue with icubGui non controlled DoF in a port)
    for(size_t j =0; j<11; j++)
    {
     // leftArmData.addDouble(qj[ti*modelState->dof()+i]);
      leftArmData.addDouble(0.0);
      rightArmData.addDouble(0.0);
    }

    //left_leg
    leftLegData.clear();
    for(i=13;i<18;i++)
    {
// 	leftLegData.addDouble(qj[ti*modelState->dof()+i]);
      leftLegData.addDouble(qj[ti+i*numTSteps] );
    }

    //right_leg
    rightLegData.clear();
    for(i=19;i<24;i++)
    {
//       rightLegData.addDouble(qj[ti*modelState->dof()+i]);
      rightLegData.addDouble(qj[ti+i*numTSteps] );
    }

//     if(!torso.write(torsoData) || !leftArm.write(leftArmData) || !rightArm.write(rightArmData) || !leftLeg.write(leftLegData) || rightLeg.write(rightLegData))
//     {
//       mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to write to icubGuiports");
//     }
    if(!floatingBase.write(floatingBaseData))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to write to icubGuiports floatingBase");
    }

    if(!torso.write(torsoData) )
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to write to icubGuiports torso");
    }

    if(!leftArm.write(leftArmData))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to write to icubGuiports leftArm");
    }

    if( !rightArm.write(rightArmData))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to write to icubGuiports rightArm");
    }

    if(!leftLeg.write(leftLegData))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to write to icubGuiports leftLeg");
    }

    if(!rightLeg.write(rightLegData))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to write to icubGuiports rightLeg");
    }
    finTime = yarp::os::Time::now();
    delayTime = t[ti+1] - t[ti] - finTime + iniTime;
    if(delayTime>0)
    {
      yarp::os::Time::delay(delayTime);
      totDelayTime = totDelayTime+delayTime;
    }
    ti = ti+1;
  }

    mexPrintf("Trying to close ports\n");
    torso.close();
    leftArm.close();
    leftLeg.close();
    rightLeg.close();
    rightArm.close();



  mexPrintf("total delay time : %f\n",totDelayTime);
  return(true);
}

/*
bool ModelVisualizeTrajectory::setState(int, const mxArray*[])
{

}*/
bool ModelVisualizeTrajectory::openPortsToICubGui(void)
{
//   if(!portStatus)
//   {
 // torso.
    mexPrintf("Trying to open ports and connect\n");
    torso.open("/mexWBIDataSource/torso");
    leftArm.open("/mexWBIDataSource/left_arm");
    leftLeg.open("/mexWBIDataSource/left_leg");
    rightLeg.open("/mexWBIDataSource/right_leg");
    rightArm.open("/mexWBIDataSource/right_arm");
    floatingBase.open("/mexWBIDataSource/base");

//     if(!yarp::os::Network::connect("/mexWBIDataSource/left_leg","/iCubGui/left_leg:i") || !yarp::os::Network::connect("/mexWBIDataSource/left_arm","/iCubGui/left_arm:i") || !yarp::os::Network::connect("/mexWBIDataSource/right_leg","/iCubGui/right_leg:i") || !yarp::os::Network::connect("/mexWBIDataSource/right_arm","/iCubGui/right_arm:i") || !yarp::os::Network::connect("/mexWBIDataSource/torso","/iCubGui/torso:i"))
//     {
//       portStatus = false;
//       mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to link to icubGuiports");
//     }
    if(!yarp::os::Network::connect("/mexWBIDataSource/left_leg","/iCubGui/left_leg:i"))
    {
      portStatus = false;
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to link to icubGuiports");
    }
    if(!yarp::os::Network::connect("/mexWBIDataSource/right_leg","/iCubGui/right_leg:i"))
    {
      portStatus = false;
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to link to icubGuiports");
    }
    if(!yarp::os::Network::connect("/mexWBIDataSource/left_arm","/iCubGui/left_arm:i"))
    {
      portStatus = false;
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to link to icubGuiports");
    }
    if(!yarp::os::Network::connect("/mexWBIDataSource/right_arm","/iCubGui/right_arm:i"))
    {
      portStatus = false;
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to link to icubGuiports");
    }
    if(!yarp::os::Network::connect("/mexWBIDataSource/torso","/iCubGui/torso:i"))
    {
      portStatus = false;
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to link to icubGuiports");
    }
    if(!yarp::os::Network::connect("/mexWBIDataSource/base","/iCubGui/base:i"))
    {
      portStatus = false;
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:networkerror","Unable to link to icubGuiports");
    }
    portStatus = true;
//   }
  mexPrintf("Connection Success!\n");
  return(portStatus);

}

