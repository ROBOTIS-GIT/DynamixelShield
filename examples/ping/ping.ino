#include <DynamixelShield.h>

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  dxl.begin(1000000, DXL_PACKET_VER_2_0);
  delay(1000);
  dxl.println("ping..");
}

void loop() {
  // put your main code here, to run repeatedly:

  if (dxl.ping() == true)
  {
    dxl.print("dxl cnt : ");
    dxl.println(dxl.getDxlCount());
    for (int i=0; i<dxl.getDxlCount(); i++)
    {
      dxl.print("  Index : ");
      dxl.print(i);
      dxl.print("  ID : ");
      dxl.println(dxl.getDxlID(i));
      dxl.ledOn(dxl.getDxlID(i));
    }
  }
  delay(500);
}
