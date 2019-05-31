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

/* Predefined RC-100 button key value
* Please refer to Remote Controller Code Map(http://emanual.robotis.com/docs/en/parts/communication/rc-100/#code-map)
* and Remote Controller Packet(http://emanual.robotis.com/docs/en/parts/communication/rc-100/#communication-packet)
*
#define RC100_BTN_U   (1)
#define RC100_BTN_D   (2)
#define RC100_BTN_L   (4)
#define RC100_BTN_R   (8)
#define RC100_BTN_1   (16)
#define RC100_BTN_2   (32)
#define RC100_BTN_3   (64)
#define RC100_BTN_4   (128)
#define RC100_BTN_5   (256)
#define RC100_BTN_6   (512)
*/

#include <DynamixelShield.h>

#ifdef SoftwareSerial_h
RobotisRemoteController rc100;
#else
RobotisRemoteController rc100(Serial1);
#endif
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Use UART port of DYNAMIXEL Shield to communicate with ROBOTIS Remote Controller.
  rc100.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t rc100_data;

  if (rc100.availableData() > 0)
  {
    rc100_data = rc100.readData();
    
    switch(rc100_data)
    {
      case RC100_BTN_U:
        Serial.println("RC100 button U");
        break;

      case RC100_BTN_D:
        Serial.println("RC100 button D");
        break;

      case RC100_BTN_L:
        Serial.println("RC100 button L");
        break;

      case RC100_BTN_R:
        Serial.println("RC100 button R");
        break;        

      case RC100_BTN_1:
        Serial.println("RC100 button 1");
        break;

      case RC100_BTN_2:
        Serial.println("RC100 button 2");
        break;        

      case RC100_BTN_3:
        Serial.println("RC100 button 3");
        break;

      case RC100_BTN_4:
        Serial.println("RC100 button 4");
        break;        

      case RC100_BTN_5:
        Serial.println("RC100 button 5");
        break;

      case RC100_BTN_6:
        Serial.println("RC100 button 6");
        break;
    }
  }
}
