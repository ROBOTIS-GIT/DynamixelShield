#include <DynamixelShield.h>

DynamixelShield dxl;


static uint8_t state_mode = 0;
static uint32_t pre_time;
static uint32_t pre_time_delay;

  
void setup() {
  // put your setup code here, to run once:
  dxl.begin(1000000, DXL_PACKET_VER_2_0);
  delay(1000);
  dxl.ping();
  dxl.torqueOff(DXL_ALL_ID);
  dxl.setWheelMode(1);
  dxl.setWheelMode(2);
  dxl.torqueOn(DXL_ALL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (dxl.getDxlCount() > 0)
  {
    switch(state_mode)
    {
      case 0:
        dxl.setGoalSpeed(1, 100);
        dxl.setGoalSpeed(2, 100);
        state_mode = 1;
        pre_time = millis();
        break;

      case 1:
        if (millis()-pre_time >= 2000)
        {
          state_mode = 2;
        }
        break;

      case 2:
        dxl.setGoalSpeed(1, -100);
        dxl.setGoalSpeed(2, -100);
        state_mode = 3;
        pre_time = millis();
        break;

      case 3:
        if ((millis()-pre_time) >= 2000)
        {
          state_mode = 0;
        }        
        break;        
    }

    if (millis()-pre_time_delay >= 50)
    {
      pre_time_delay = millis();
      dxl.print(dxl.getCurSpeed(1));
      dxl.print(" ");
      dxl.println(dxl.getCurSpeed(2));
    }
  }
}
