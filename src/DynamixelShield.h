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
 * Version: 0.0.4
 * Authors: Hancheol Cho (Baram) 
 *          KyungWan Ki  (Kei)
 */

#ifndef DYNAMIXEL_SHIELD_H_
#define DYNAMIXEL_SHIELD_H_

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "util/RobotisRemoteController.h"

#ifdef ARDUINO_AVR_UNO
  #define DXL_SERIAL   Serial
#elif ARDUINO_AVR_MEGA2560
  #define DXL_SERIAL   Serial
#else
  #define DXL_SERIAL   Serial1
#endif

const int DIR_PIN = 2;

class DynamixelShield : public Dynamixel2Arduino
{
public:
  DynamixelShield(HardwareSerial& port = DXL_SERIAL, int dir_pin = DIR_PIN);
  ~DynamixelShield();

private:

};

#endif 
