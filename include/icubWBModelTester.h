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

#include<string>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
using namespace wbi;
using namespace wbiIcub;

#ifndef	ICUBWBMODELTESTER_H
#define ICUBWBMODELTESTER_H

//extern std::pair< iterator, iterator > r;
class IcubWBModelTester
{
public:
IcubWBModelTester();
void runTest(float);
~IcubWBModelTester();
private:
  std::string robotName;
  std::string localName;
  //iWholeBodyModel *icub ;
  icubWholeBodyModel *icubModel;
};

#endif // ICUBWHOLEBODYMODELTESTER_H
