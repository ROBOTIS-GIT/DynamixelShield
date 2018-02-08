#include <DynamixelShield.h>

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  dxl.begin(1000000, DXL_PACKET_VER_2_0);
  delay(1000);
  dxl.ping();
}

void loop() {
  // put your main code here, to run repeatedly:

  if (dxl.getDxlCount() > 0)
  {
    dxl.ledOn(1);
    delay(500);
    dxl.ledOff(1);
    delay(500);    
  }  
}
