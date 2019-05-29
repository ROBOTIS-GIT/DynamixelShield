/*
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

RobotisRemoteController rc100;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  rc100.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t rc100_data;

  if (rc100.available() > 0)
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
