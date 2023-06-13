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

/* ControlTableItem (1.0 and 2.0)
  The naming convention for the ControlTable Item macro is as follows:
    From the name specified in the Control Table of each model(please refer to the e-manual (http://emanual.robotis.com)),
    the alphabet is replaced with capital letters
    and the space(" ") is replaced with an underscore(_).
    
  MODEL_NUMBER,
  MODEL_INFORMATION,
  FIRMWARE_VERSION,
  PROTOCOL_VERSION,
  ID,
  SECONDARY_ID,
  BAUD_RATE,
  DRIVE_MODE,
  CONTROL_MODE,
  OPERATING_MODE,
  CW_ANGLE_LIMIT,
  CCW_ANGLE_LIMIT,
  TEMPERATURE_LIMIT,
  MIN_VOLTAGE_LIMIT,
  MAX_VOLTAGE_LIMIT,
  PWM_LIMIT,
  CURRENT_LIMIT,
  VELOCITY_LIMIT,
  MAX_POSITION_LIMIT,
  MIN_POSITION_LIMIT,
  ACCELERATION_LIMIT,
  MAX_TORQUE,
  HOMING_OFFSET,
  MOVING_THRESHOLD,
  MULTI_TURN_OFFSET,
  RESOLUTION_DIVIDER,
  EXTERNAL_PORT_MODE_1,
  EXTERNAL_PORT_MODE_2,
  EXTERNAL_PORT_MODE_3,
  EXTERNAL_PORT_MODE_4,
  STATUS_RETURN_LEVEL,
  RETURN_DELAY_TIME,
  ALARM_LED,
  SHUTDOWN,

  TORQUE_ENABLE,
  LED,
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  REGISTERED_INSTRUCTION,
  HARDWARE_ERROR_STATUS,
  VELOCITY_P_GAIN,
  VELOCITY_I_GAIN,
  POSITION_P_GAIN,
  POSITION_I_GAIN,
  POSITION_D_GAIN,
  FEEDFORWARD_1ST_GAIN,
  FEEDFORWARD_2ND_GAIN,
  P_GAIN,
  I_GAIN,
  D_GAIN,
  CW_COMPLIANCE_MARGIN,
  CCW_COMPLIANCE_MARGIN,
  CW_COMPLIANCE_SLOPE,
  CCW_COMPLIANCE_SLOPE,
  GOAL_PWM,
  GOAL_TORQUE,
  GOAL_CURRENT,
  GOAL_POSITION,
  GOAL_VELOCITY,
  GOAL_ACCELERATION,
  MOVING_SPEED,
  PRESENT_PWM,
  PRESENT_LOAD,
  PRESENT_SPEED,
  PRESENT_CURRENT,
  PRESENT_POSITION,
  PRESENT_VELOCITY,
  PRESENT_VOLTAGE,
  PRESENT_TEMPERATURE,
  TORQUE_LIMIT,
  REGISTERED,
  MOVING,
  LOCK,
  PUNCH,
  CURRENT,
  SENSED_CURRENT,
  REALTIME_TICK,
  TORQUE_CTRL_MODE_ENABLE,
  BUS_WATCHDOG,
  PROFILE_ACCELERATION,
  PROFILE_VELOCITY,
  MOVING_STATUS,
  VELOCITY_TRAJECTORY,
  POSITION_TRAJECTORY,
  PRESENT_INPUT_VOLTAGE,
  EXTERNAL_PORT_DATA_1,
  EXTERNAL_PORT_DATA_2,
  EXTERNAL_PORT_DATA_3,
  EXTERNAL_PORT_DATA_4,
*/

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

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
// MX and AX servos use DYNAMIXEL Protocol 1.0 by default.
// to use MX and AX servos with this example, change the following line to: const float DXL_PROTOCOL_VERSION = 1.0;
  dxl.setPortProtocolVersion(2.0);
  dxl.begin(57600);
  dxl.scan();

  // Turn on torque
  dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);

  /* set velocity(speed) mode */
  // MX2.0, X serise
  dxl.writeControlTableItem(OPERATING_MODE, DXL_ID, 1);
  // AX~MX
  dxl.writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID, 0);
  dxl.writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID, 0);
  // XL320
  dxl.writeControlTableItem(CONTROL_MODE, DXL_ID, 1);
  
  // Turn on torque
  dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
}

void loop() {
  // put your main code here, to run repeatedly:

  /* Write & read velocity(speed) data */
  // MX2.0, X serise
  if(dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 200)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_VELOCITY, DXL_ID));
  }
  // AX~MX & XL320
  if(dxl.writeControlTableItem(MOVING_SPEED, DXL_ID, 200)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_SPEED, DXL_ID));
  }

  delay(2000);  

  // MX2.0, X serise
  if(dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 0)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_VELOCITY, DXL_ID));
  }
  // AX~MX & XL320
  if(dxl.writeControlTableItem(MOVING_SPEED, DXL_ID, 0)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_SPEED, DXL_ID));
  }

  delay(2000);  
}
