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

DynamixelShield dxl;


  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  dxl.rc100.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t rc100_data;
  
  if (dxl.rc100.available() > 0)
  {
    rc100_data = dxl.rc100.readData();
    
    Serial.println(rc100_data);
    switch(rc100_data)
    {
      case RC100_BTN_U:
        Serial.println("RC100_BTN_U");
        break;

      case RC100_BTN_D:
        Serial.println("RC100_BTN_D");
        break;        
    }
  }
}
