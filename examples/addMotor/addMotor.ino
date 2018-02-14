/*
#define M_AX12    ((uint8_t)0)
#define M_AX18    ((uint8_t)1)
#define M_XL320   ((uint8_t)2)
#define M_XL430   ((uint8_t)3)
#define M_XM430   ((uint8_t)4)
#define M_XM540   ((uint8_t)5)
*/


#include <DynamixelShield.h>

DynamixelShield dxl;


  
void setup() {
  // put your setup code here, to run once:
  dxl.begin(1000000);
  delay(1000);
  dxl.addMotor(1,  M_XL430);
  dxl.addMotor(2,  M_XL430);
  dxl.addMotor(15, M_AX12);
  dxl.torqueOn(1);
  dxl.torqueOn(2);
  dxl.torqueOn(15);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (dxl.getDxlCount() > 0)
  {
    dxl.setGoalAngle(1,  0);
    dxl.setGoalAngle(2,  0);
    dxl.setGoalAngle(15, 0);
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);
    dxl.setGoalAngle(1,  900);
    dxl.setGoalAngle(2,  900);    
    dxl.setGoalAngle(15, 900);
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);
    dxl.setGoalAngle(1,  0);
    dxl.setGoalAngle(2,  0);
    dxl.setGoalAngle(15, 0);    
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);
    dxl.setGoalAngle(1, -900);
    dxl.setGoalAngle(2, -900);    
    dxl.setGoalAngle(15,-900);
    dxl.println(dxl.getGoalAngle(1));
    delay(1000);    
  }
}
