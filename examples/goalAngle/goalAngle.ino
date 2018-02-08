#include <DynamixelShield.h>

DynamixelShield dxl;


  
void setup() {
  // put your setup code here, to run once:
  dxl.begin(1000000, DXL_PACKET_VER_2_0);
  delay(1000);
  dxl.ping();
  dxl.torqueOn(DXL_ALL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (dxl.getDxlCount() > 0)
  {
    dxl.setGoalAngle(1, 0);
    dxl.setGoalAngle(2, 0);
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);
    dxl.setGoalAngle(1, 900);
    dxl.setGoalAngle(2, 900);    
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);
    dxl.setGoalAngle(1, 0);
    dxl.setGoalAngle(2, 0);    
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);
    dxl.setGoalAngle(1, -900);
    dxl.setGoalAngle(2, -900);    
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);    
  }
}
