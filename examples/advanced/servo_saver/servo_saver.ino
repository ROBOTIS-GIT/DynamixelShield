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

// Example Environment
//
// - DYNAMIXEL: X series providing Current-based Position Control mode 
//              ID = 1, Baudrate = 57600bps, DYNAMIXEL Protocol 2.0
// - Controller: Arduino MKR ZERO
//               DYNAMIXEL Shield for Arduino MKR
// - https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/
//
// Author: David Park

// NOTE:
// * Make sure this example is designed based on DYNAMIXEL Protocol 2.0 only. 
// * If your DYNAMIXEL does not feature current sensing mode, this example may not be your option.   

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
const float DXL_PROTOCOL_VERSION = 2.0;

int64_t std_goal_pos = 0;
int64_t pre_pos = 0;

// Set PID gain
int32_t p_gain = 100;
int32_t i_gain = 0;
int32_t d_gain = 0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(57600);
  
  while (!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalCurrent(DXL_ID, 100);   

  // Set Reference Position 
  dxl.setGoalPosition(DXL_ID, 2048); 

  dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, p_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, d_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, i_gain);
}

void loop() {
  // put your main code here, to run repeatedly:
  DEBUG_SERIAL.print("Desired_Position:"); 
  DEBUG_SERIAL.print(dxl.readControlTableItem(GOAL_POSITION, 1)); 
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print("Present_Position:"); 
  DEBUG_SERIAL.print(dxl.getPresentPosition(DXL_ID)); 
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println();
  
  delay(10);

}
