/*******************************************************************************
* Copyright 2022 ROBOTIS CO., LTD.
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

// NOTE: Note that "pid.ino" example is mainly designed for DYNAMIXEL with Protocol 2.0. 

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

unsigned long timer = 0;

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

int32_t pos_profile[3] = {1200, 1600};
int8_t direction = 0;

// Set Position P.I.D Gains. 
uint16_t pos_d_gain = 0;
uint16_t pos_i_gain = 0;
uint16_t pos_p_gain = 0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(57600);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  
  // Set Position P.I.D Gains.
  dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, pos_p_gain);  
  dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, pos_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, pos_d_gain);  

  while(!DEBUG_SERIAL);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Read Present Position and see it reaches for the desired position (Use a Serial Plotter)
  while(1){
    DEBUG_SERIAL.print("Desired_Position(raw):"); 
    DEBUG_SERIAL.print(dxl.readControlTableItem(GOAL_POSITION, 1)); 
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print("Present_Position(raw):"); 
    DEBUG_SERIAL.print(dxl.getPresentPosition(DXL_ID)); 
    DEBUG_SERIAL.print(","); 
    DEBUG_SERIAL.println();
    delay(10);

    if (millis()-timer >= 2000){ 
      dxl.setGoalPosition(DXL_ID, pos_profile[direction]); 
      timer = millis();
      break;
    }
  }
  if(direction >=1){
    direction = 0;
  }else{
    direction = 1;
  }

}