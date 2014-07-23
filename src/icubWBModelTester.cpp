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
#include <iostream>
#include <yarp/os/Time.h>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/os/Property.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>

#include <wbiIcub/wholeBodyInterfaceIcub.h>

#include <wbi/wbiUtil.h>

#include <stdio.h>
#include <math.h>
#include <string>
#include <cstdlib>

#include <iostream>
#include <typeinfo>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace wbiIcub;
/*
// #include <yarp/os/Network.h>
// #include <yarp/os/Time.h>
// #include <yarp/os/Property.h>
// 
// #include <yarp/os/Property.h>
// 
// #include <yarp/math/Math.h>
 #include <yarp/sig/Vector.h>
 #include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>
#include <wbi/wbi.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <wbi/wbiUtil.h>

// #include <stdio.h>
// #include <math.h>
#include <string>
// #include <cstdlib>


// #include <typeinfo>
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
// using namespace std;
using namespace wbi;
using namespace wbiIcub;

using namespace yarp::os;*/

#include "icubWBModelTester.h"
#include <iostream>

IcubWBModelTester::IcubWBModelTester()
{ 
  std::cout<<"IcubWBModelTestr object created! \n";
  
  std::string localName = "wbiTest";
  std::string robotName = "icub";
    
    
  //  std::cout<<"iDynTree version"<<iCub::iDynTree::iCubTree_version_tag(2,1,true);
  
   // iWholeBodyInterface *icub;// = new icubWholeBodyInterface(localName.c_str());//(localName.c_str(), robotName.c_str(),iCub::iDynTree::iCubTree_version_tag(2,1,true));
  icubModel = new icubWholeBodyModel(localName.c_str(),robotName.c_str(),iCub::iDynTree::iCubTree_version_tag(2,2,true));  
  
}

void IcubWBModelTester::runTest(float delayVal)
{ 
  int nr_of_considered_joints = (int)icubModel->getJointList().size();
  int nr_of_activated_joints = 0;
  yarp::sig::Vector theta = 2*M_PI*yarp::math::Rand::vector(nr_of_considered_joints);
  yarp::sig::Matrix mass_matrix(6+nr_of_considered_joints,6+nr_of_considered_joints);
   Time::delay(0.5); 
    std::cout << "Creating icubWholeBodyModel with robotName " << robotName << " and localName " << localName << std::endl;
    
   icubModel->wbiIcub::icubWholeBodyModel::addJoints(wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
//   FOR_ALL_OF(itBp, itJ, wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS) {
//   //if( nr_of_activated_joints < nr_of_considered_joints && yarp::math::Rand::scalar() < threshold ) {
//       if( icubModel->addJoint(LocalId(itBp->first,*itJ)) ) {
// 	  nr_of_activated_joints++;
//       }
//   }
  double minJointLimits[icubModel->getDoFs()];
  double maxJointLimits[icubModel->getDoFs()];
  icubModel->getJointLimits(minJointLimits,maxJointLimits);

  for(int i = 0; i<icubModel->getDoFs(); i++)
  {
      std::cout<<" Joint :"<<i<<", MinLimit"<<minJointLimits[i]<<", MaxLimit"<<maxJointLimits[i]<<std::endl;
  }
}
IcubWBModelTester::~IcubWBModelTester()
{
  delete icubModel;
  std::cout<<"icubWholeBodyModel destroyed \n";
   
}
