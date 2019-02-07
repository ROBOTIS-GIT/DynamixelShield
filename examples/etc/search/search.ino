#include <DynamixelShield.h>

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:  
  uint32_t baud_tbl[6] = { 9600, 57141, 57600, 115200, 1000000, 2000000 };
  
  dxl.begin(1000000, DXL_PACKET_VER_2_0);

  dxl.println("DXL 1.0 Searching...");  
  for (int i=0; i<6; i++)
  {
    dxl.begin(baud_tbl[i], DXL_PACKET_VER_1_0);
    delay(100);
    dxl.print(baud_tbl[i]);  
    dxl.println("bps ..");
    
    if (dxl.ping() == true)
    {
      dxl.print("  dxl cnt : ");
      dxl.println(dxl.getDxlCount());
      for (int i=0; i<dxl.getDxlCount(); i++)
      {
        dxl.print("    Index : ");
        dxl.print(i);
        dxl.print("    ID : ");
        dxl.println(dxl.getDxlID(i));
        dxl.ledOn(dxl.getDxlID(i));
        delay(200);
        dxl.ledOff(dxl.getDxlID(i));        
      }
    }
  }

  dxl.println("\r\nDXL 2.0 Searching...");  
  for (int i=0; i<6; i++)
  {
    dxl.begin(baud_tbl[i], DXL_PACKET_VER_2_0);
    delay(100);
    dxl.print(baud_tbl[i]);  
    dxl.println("bps ..");
    
    if (dxl.ping() == true)
    {
      dxl.print("  dxl cnt : ");
      dxl.println(dxl.getDxlCount());
      for (int i=0; i<dxl.getDxlCount(); i++)
      {
        dxl.print("    Index : ");
        dxl.print(i);
        dxl.print("    ID : ");
        dxl.println(dxl.getDxlID(i));
        dxl.ledOn(dxl.getDxlID(i));
        delay(200);
        dxl.ledOff(dxl.getDxlID(i));        
      }
    }
  }  
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
}
