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

/** Please refer to each DYNAMIXEL eManual(http://emanual.robotis.com) for supported Operating Mode
 * Operating Mode
 *  1. OP_POSITION                (Position Control Mode in protocol2.0, Joint Mode in protocol1.0)
 *  2. OP_VELOCITY                (Velocity Control Mode in protocol2.0, Speed Mode in protocol1.0)
 *  3. OP_PWM                     (PWM Control Mode in protocol2.0)
 *  4. OP_EXTENDED_POSITION       (Extended Position Control Mode in protocol2.0, Multi-turn Mode(only MX series) in protocol1.0)
 *  5. OP_CURRENT                 (Current Control Mode in protocol2.0, Torque Mode(only MX64,MX106) in protocol1.0)
 *  6. OP_CURRENT_BASED_POSITION  (Current Based Postion Control Mode in protocol2.0 (except MX28, XL430))
 */

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
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Position Control Mode in protocol2.0, Joint Mode in protocol1.0
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 512)){
    delay(1000);
    DEBUG_SERIAL.print("Present Position : ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID)); DEBUG_SERIAL.println();
  }

  // Velocity Contorl Mode in protocol2.0, Speed Mode in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalVelocity(DXL_ID, 128)){
    delay(1000);
    DEBUG_SERIAL.print("Present Velocity : ");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID)); DEBUG_SERIAL.println();
  }

  // Extended Position Contorl Mode in protocol2.0, Multi-turn Mode(only MX series) in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 4096)){
    delay(1000);
    DEBUG_SERIAL.print("Present Extended Position : ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID)); DEBUG_SERIAL.println();
  }

  // PWM Contorl Mode in protocol2.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPWM(DXL_ID, 200)){
    delay(1000);
    DEBUG_SERIAL.print("Present PWM : ");
    DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID)); DEBUG_SERIAL.println();
  }
  
  // Current Contorl Mode in protocol2.0, Torque Mode(only MX64,MX106) in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalCurrent(DXL_ID, 256)){
    delay(1000);
    DEBUG_SERIAL.print("Present Current : ");
    DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID)); DEBUG_SERIAL.println();
  }
  
  // Current Based Postion Contorl Mode in protocol2.0 (except MX28, XL430)
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 8192)){
    delay(1000);
    DEBUG_SERIAL.print("Present Current Based Position : ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID)); DEBUG_SERIAL.println();
  }
}
