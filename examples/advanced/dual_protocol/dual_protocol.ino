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

/* NOTE: When using multiple Protocols, use only one baudrate */

#include <DynamixelShield.h>

#ifdef ARDUINO_AVR_UNO
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); //DYNAMIXEL Shield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif ARDUINO_AVR_MEGA2560
  #define DEBUG_SERIAL Serial1
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION_1 = 1.0;
const float DXL_PROTOCOL_VERSION_2 = 2.0;

DynamixelShield dxl_1;  //Create Protocol 1.0 class
DynamixelShield dxl_2;  //Create Protocol 2.0 class

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps.
  dxl_1.begin(57600);
  // Set Port Protocol Version 1.0 for DXL ID 1.
  dxl_1.setPortProtocolVersion(DXL_PROTOCOL_VERSION_1);
  // Get DYNAMIXEL information
  dxl_1.ping(DXL_ID_1);

  // Set Port baudrate to 57600bps.
  dxl_2.begin(57600);
  // Set Port Protocol Version 2.0 for DXL ID 2.
  dxl_2.setPortProtocolVersion(DXL_PROTOCOL_VERSION_2);
  // Get DYNAMIXEL information
  dxl_2.ping(DXL_ID_2);

  // Turn off torque before configuring operating mode(EEPROM area)
  dxl_1.torqueOff(DXL_ID_1);
  dxl_1.setOperatingMode(DXL_ID_1, OP_POSITION);
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl_2.torqueOn(DXL_ID_2);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Set Goal Position in RAW value
  dxl_1.setGoalPosition(DXL_ID_1, 512);
  // Set Goal Position in DEGREE value
  dxl_2.setGoalPosition(DXL_ID_2, 5.7, UNIT_DEGREE);
  
  delay(1000);
  // Print present position in raw value
  DEBUG_SERIAL.print("ID 1, Protocol 1.0, Present Position(raw) : ");
  DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1));
  
  // Print present position in degree value
  DEBUG_SERIAL.print("ID 2, Protocol 2.0, Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_DEGREE));
  delay(1000);

  dxl_1.setGoalPosition(DXL_ID_1, 0);
  // Set Goal Position in DEGREE value
  dxl_2.setGoalPosition(DXL_ID_2, 25.7, UNIT_DEGREE);

  delay(1000);
  // Print present position in raw value
  DEBUG_SERIAL.print("ID 1, Protocol 1.0, Present Position(raw) : ");
  DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1));
  
  // Print present position in degree value
  DEBUG_SERIAL.print("ID 2, Protocol 2.0, Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_DEGREE));
  delay(1000);
}
