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

#define TIMEOUT 10    //default communication timeout 10ms
const uint8_t DXL_ID = 1;
// MX and AX servos use DYNAMIXEL Protocol 1.0 by default.
// to use MX and AX servos with this example, change the following line to: const float DXL_PROTOCOL_VERSION = 1.0;
const float DXL_PROTOCOL_VERSION = 2.0;
uint8_t option = 0;

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //Set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port for terminal is opened
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  bool ret = false;
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("Reboot DYNAMIXEL? [y/n]");

  DEBUG_SERIAL.read();
  while(DEBUG_SERIAL.available()==0);
  option = DEBUG_SERIAL.read();

  switch(option) {
    case 'y':
    case 'Y':
      ret = dxl.reboot(DXL_ID, TIMEOUT);
      break;
    default:
      break;
  }
  if(ret) {
    DEBUG_SERIAL.println("Reboot Succeeded!");
  } else {
    DEBUG_SERIAL.println("Reboot Failed!");
  }

  delay(1000);
}
