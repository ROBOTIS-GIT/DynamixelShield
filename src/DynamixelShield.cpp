/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* 
 * Authors: Hancheol Cho (Baram) 
 *          KyungWan Ki  (Kei)
 */

#include "DynamixelShield.h"


#ifndef SoftwareSerial_h
#pragma message("\r\nWarning : You can't use the RC100 function, because this board doesn't have SoftwareSerial.h")
#endif



DynamixelShield::DynamixelShield(HardwareSerial& port, int dir_pin)
  : Dynamixel2Arduino(port, dir_pin)
{}

DynamixelShield::~DynamixelShield()
{}


