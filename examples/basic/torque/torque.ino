#include <DynamixelShield.h>

const uint8_t DXL_ID = 1;

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  dxl.begin(1000000);
  dxl.ping(DXL_ID);  
}

void loop() {
  // put your main code here, to run repeatedly:
  dxl.torqueOn(DXL_ID);
  delay(2000);
  dxl.torqueOff(DXL_ID);
  delay(2000);
}
