#include "modelcomponent.h"
//#include "robotwholebodymodel.h"
#include <iostream>
#include<wbiIcub/icubWholeBodyModel.h>
#include "modelstate.h"

using namespace mexWBIComponent;


ModelComponent::ModelComponent(wbi::iWholeBodyModel *w)
{
//   numArguments = n;
//   argumentMexArray = m;
  robotModel =  w; 
  
//   modelState = ModelState::getInstance(robotModel->getDoFs());
  
  numDof = robotModel->getDoFs();
//   q = new double(numDof-6);
//   dq = new double(numDof-6);
//   dxb = new double(6);  
}


ModelComponent::~ModelComponent() 
{
}

ModelComponent* ModelComponent::getInstance()
{
  return(NULL);
}

// void ModelComponent::setArguments(int n, mxArray* m)
// {
//   numArguments = n;
//   argumentMexArray = m;
// }

//   bool set_q(mxArray *[]);
//   bool set_qdd(mxArray *[]);
//   bool set_v(mxArray *[]);
//   bool set_tau(mxArray *[]);
//   
/*
bool ModelComponent::set_q(mxArray* prhs)
{
  
  
  q = mxGetPr(prhs);
  
  return(true);
}
bool ModelComponent::set_qdd(mxArray* prhs)
{
  retVal = true;
  if(numDof = mxGetM(prhs[1])
  {
    
    
  }qDD = mxGetPr(prhs);
  return(true);
}
bool ModelComponent::set_v(mxArray* prhs)
{
  
  v = mxGetPr(prhs);
  return(true);
}*/
/*bool ModelComponent::set_tau(mxArray* m[])
{
  return(true);
}*/

