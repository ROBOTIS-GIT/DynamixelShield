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

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB  
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID = 1;
// MX and AX servos use DYNAMIXEL Protocol 1.0 by default.
// to use MX and AX servos with this example, change the following line to: const float DXL_PROTOCOL_VERSION = 1.0;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
   
  // Please refer to e-Manual(http://emanual.robotis.com) for available range of value. 
  // Set Goal PWM using RAW unit
  dxl.setGoalPWM(DXL_ID, 300);
  delay(1000);
  // Print present PWM
  DEBUG_SERIAL.print("Present PWM(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID));
  delay(1000);

  // Set Goal PWM using percentage (-100.0 [%] ~ 100.0 [%])
  dxl.setGoalPWM(DXL_ID, -40.8, UNIT_PERCENT);
  delay(1000);
  DEBUG_SERIAL.print("Present PWM(ratio) : ");
  DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID, UNIT_PERCENT));
  delay(1000);
}
