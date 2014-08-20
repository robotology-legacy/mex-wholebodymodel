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


// global includes

// library includes
#include <wbi/iWholeBodyModel.h>
#include <mex.h>

// #include <yarp/math/Rand.h>

// local includes
#include "modelupdatestate.h"
#include "modelstate.h"


using namespace mexWBIComponent;
  
//   virtual const int numReturns();
//   virtual void display(int, mxArray *[]);
//   virtual void compute(int, mxArray *[]);
//   
//   ~ModelStateSetter();
// private:
//   
//   static ModelStateSetter * modelStateSetter;

ModelUpdateState *ModelUpdateState::modelUpdateState;

ModelUpdateState::ModelUpdateState() : ModelComponent(3,3,0)
{
  //numDof = robotModel->getDoFs();
#ifdef DEBUG
  mexPrintf("ModelUpdateState constructed \n");
#endif
  
}

ModelUpdateState * ModelUpdateState::getInstance()
{
  if(modelUpdateState == NULL)
  {
    modelUpdateState = new ModelUpdateState;
  }
  
  return(modelUpdateState);
}


ModelUpdateState::~ModelUpdateState()
{

}

bool ModelUpdateState::compute(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelUpdateState\n");
#endif
  bool retVal = setState(nrhs,prhs);
   return(retVal);
//   return(true);

}

bool ModelUpdateState::computeFast(int nrhs, const mxArray *prhs[])
{
#ifdef DEBUG
  mexPrintf("Trying to compute ModelUpdateState\n");
#endif
  robotModel = modelState->robotModel();
  bool retVal = setState(nrhs,prhs);
   return(retVal);
//   return(true);

}

// bool ModelStateUpdater::display(int nrhs, const mxArray* prhs[])
// {
//   mexPrintf("Trying to display ModelStateUpdater\n");
//   bool retState = setState(nrhs,prhs);
//   return(retState);
// //   return(true);
// }

bool ModelUpdateState::allocateReturnSpace(int nlhs, mxArray* plhs[])
{
//   bool retVal = false;
// //   if(nlhs==0)
// //   {
//     plhs = NULL;//mxCreateNumericMatrix(1,1, mxINT8_CLASS, mxREAL);//
//     retVal = true;
// //   }
//   //int *retVal =  mxGetPr(plhs[0]);
//   //retVal
//   return(retVal);
  
#ifdef DEBUG
  mexPrintf("Trying to allocate memory in ModelUpdateState (nothing to do really)\n");
#endif
  return(true);

}

bool ModelUpdateState::setState(int nrhs, const mxArray* prhs[])
{
//   bool retVal = false;
//   if(nrhs == 4)
//   {
//     retVal = true;
//     
//     mexPrintf("the sizes of args are %dx%d, %dx%d, %dx%d \n",mxGetM(prhs[1]),mxGetN(prhs[1]),mxGetM(prhs[2]),mxGetN(prhs[2]),mxGetM(prhs[3]),mxGetN(prhs[3]) ); 
    if(mxGetM(prhs[1])!=numDof || mxGetN(prhs[1])!=1 || mxGetM(prhs[2])!=numDof ||mxGetN(prhs[2])!=1 || mxGetM(prhs[3])!=6 || mxGetN(prhs[3])!=1)
    {
//       retVal = false;
//       mexPrintf("Malformed state vectors, check dimensions\n");
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Malformed state dimensions / inputs");
    }
//     else
//     {
#ifdef DEBUG
       mexPrintf("Updating state\n");
#endif
     robotModel = modelState->robotModel();
//        retVal = true;
     //double q_temp[numDof],dq_temp[numDof],dxb_temp[6];
     double *q_temp,*dq_temp,*dxb_temp;
      q_temp = (double *)mxGetPr(prhs[1]);
      dq_temp = (double *)mxGetPr(prhs[2]);
      dxb_temp = (double *)mxGetPr(prhs[3]);        
     
//       q_temp = new double(modelState->dof());
//       dq_temp = new double(modelState->dof());
//       dxb_temp = new double(6);
//       
      robotModel->computeH(q_temp,wbi::Frame(),ROBOT_BASE_FRAME_LINK, H_base_wrfLink);
  
      H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
      xB.set4x4Matrix (H_w2b.data());
      
      modelState->setState(q_temp,dq_temp,dxb_temp,xB);
            
#ifdef DEBUG
      mexPrintf("Updated state : (q,dq)\n");

      int numDof = modelState->dof();
      
      for(int i = 0; i < numDof ; i++)
      {
	//q_temp[i] = yarp::math::Rand::scalar(-1,1);
	//dq_temp[i] = yarp::math::Rand::scalar(-1,1);
	
//  	q[i] = q_temp[i];
// 	dq[i] = dq_temp[i];
	mexPrintf("%d:[%lf,%lf]  ",i,q_temp[i],dq_temp[i]);
      }
      mexPrintf("\n(dxb) \n");
      for(int i = 0;i<6 ;i++)
      {
// 	dxb[i] = dxb_temp[i];
	//dxb_temp[i] = yarp::math::Rand::scalar(-1,1);
	mexPrintf("%d:[%lf] ",i,dxb_temp[i]);
      }
//      modelState->setState(q_temp,dq_temp,dxb_temp,updateBaseFrame(q_temp) );
//      ModelComponent::setState(q_temp,dq_temp,dxb_temp,updateBaseFrame(q_temp) );
      //(q);
      mexPrintf("Updated State\n");
#endif
//      // delete(q_temp);delete(dq_temp);delete(dxb_temp);
//     //} 
//     
//   }
//   else
//   {
//      mexPrintf("Malformed inputs, atleast 3 more vectors of right dimensions required\n");
//   }
  return(true);
}
/*wbi::Frame ModelStateUpdater::updateBaseFrame(double *q)
{
  mexPrintf("Trying to update robot base frame position\n");
  int LINK_FOOT_WRF;
  wbi::Frame H_base_wrfLink,xB;
  robotModel->getLinkId ("l_sole", LINK_FOOT_WRF);
  robotModel->computeH(q,wbi::Frame(),LINK_FOOT_WRF, H_base_wrfLink);
  
  H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
  xB.set4x4Matrix (H_w2b.data());
  
  return(xB);
//   mexPrintf("Reference frame rototranslation is set\n");
//   
}*//*
const int ModelStateUpdater::numReturns()
{
  return(numReturnArguments);
}*/
