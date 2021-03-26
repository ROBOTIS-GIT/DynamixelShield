/*******************************************************************************
  Copyright 2021 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

#include <SPI.h>
#include <ArduinoMCP2515.h>
#include <DynamixelShield.h>

#undef max
#undef min
#include <algorithm>

// The data structure to store transmitted data. 
typedef struct {
  uint8_t ucID;
  uint8_t ucLength;
  uint8_t pucBuf[8];
} CANData_t;

// Enum to define the value to the field of the Instruction Packet.
enum {
  CAN_DXL_PING          = 0x01,
  CAN_DXL_READ,
  CAN_DXL_WRITE,
  CAN_DXL_REG_WRITE,
  CAN_DXL_ACTION,
  CAN_DXL_FACTORY_RESET,
  CAN_DXL_REBOOT        = 0x08,
  CAM_DXL_CLEAR         = 0x10,
  CAN_DXL_STATUS        = 0x55
};

// Function declaration for the control of MCP2515.

void    spi_select           ();
void    spi_deselect         ();
uint8_t spi_transfer         (uint8_t const);
void    onExternalEvent      ();
void    onReceiveBufferFull  (uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

/**************************************************************************************
Variables to configure by a user.
 **************************************************************************************/

// Use of serial debug. 

// In use of the serial debug, be sure to transmit and receive data in over 5 ms communication period under the CAN communication, otherwise the system may not work properly. 

// #define USING_DEBUG_SERIAL

#ifdef USING_DEBUG_SERIAL
const uint32_t  SERIAL_BUADRATE = 9600;
#endif

const uint8_t MKR_CAN_MCP2515_CS_PIN  = 3; // Define the CS pin number for SPI. 

const uint8_t MKR_CAN_MCP2515_INT_PIN = 7; // Define the external interrupt pin number for the received event. 
const uint8_t RECV_BUFFER_SIZE = 30;       // Define buffer size to receive. 
const CanBitRate CAN_BUADRATE = CanBitRate::BR_1000kBPS_16MHZ; // Communication Speed for CAN
//CanBitRate::BR_125kBPS
//CanBitRate::BR_250kBPS
//CanBitRate::BR_500kBPS
//CanBitRate::BR_1000kBPS

// Define the DYNAMIXEL Protocol to use.
const float DXL_PROTOCOL_VERSION = 2.0;

// Define the baud rate of DYNAMIXEL in use. Note that over 1 Mbps may cause unexpected issues. 
const uint32_t  DXL_BUADRATE = 1000000; 

/***************************************************************************************/


DynamixelShield dxl;
ArduinoMCP2515 mcp2515(spi_select, spi_deselect, spi_transfer,  micros, onReceiveBufferFull, nullptr);

// Buffer to receive the transmitted data. 
CANData_t _pDataBuf[RECV_BUFFER_SIZE];
uint8_t _ucDataBufCntWrite;
uint8_t _ucDataBufCntRead;


void setup() {
 
  // set the communication speed, and must correspond to the DYNAMIXEL’s baud rate in use. 
  dxl.begin(DXL_BUADRATE);

  // set the DYNAMIXEL Protocol. Note that the value must correspond to the protocol of DYNAMIXEL in use. 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // It Initializes the SPI communication.
  SPI.begin();
  pinMode(MKR_CAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKR_CAN_MCP2515_CS_PIN, HIGH);

  // Configure the external interrupt in order to receive the data of CAN communication.
  pinMode(MKR_CAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKR_CAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  // Configure the CAN Control IC 
  mcp2515.begin();
  mcp2515.setBitRate(CAN_BUADRATE);
  mcp2515.setNormalMode();

  
#ifdef USING_DEBUG_SERIAL

  // Initialize the USB serial port. 
  Serial.begin(SERIAL_BUADRATE);

  Serial.println("Start CAN2Dynamixel");
#endif
}



void loop() {

  uint8_t ucID = 0;
  uint8_t ucLen = 0;

  uint8_t ucDxlInstruction = 0;
  uint8_t pucStatusDataBuffer[8] = {0x55, 0,};
  uint8_t ucLength = 0;

  uint16_t usAddress = 0;
  uint16_t usDataSize = 0;
  uint8_t ucReadLength = -1;


  int8_t cResult = 0;

  // Run until the buffer is empty. 
  for ( ; _ucDataBufCntRead != _ucDataBufCntWrite; ) {

    // Get data from the receive buffer. 
    ucID = _pDataBuf[_ucDataBufCntRead].ucID;
    ucLen = _pDataBuf[_ucDataBufCntRead].ucLength;
    ucDxlInstruction = _pDataBuf[_ucDataBufCntRead].pucBuf[0];

    // Communication with DYNAMIXELs to the selected Instruction. 
    switch (ucDxlInstruction) {

      // Ping 
      case CAN_DXL_PING:
        ucLength = 5;
        cResult = DXL_Ping(ucID, pucStatusDataBuffer);
        break;

      // Read
      case CAN_DXL_READ:
        ucLength = _pDataBuf[_ucDataBufCntRead].pucBuf[3] + 2;
        cResult = DXL_Read(ucID, _pDataBuf[_ucDataBufCntRead].pucBuf, pucStatusDataBuffer);
        break;

      // Write
      case CAN_DXL_WRITE:
        ucLength = 2;
        cResult = DXL_Write(ucID, _pDataBuf[_ucDataBufCntRead].pucBuf, ucLen);
        break;

      // Reg Write
      case CAN_DXL_REG_WRITE:
        ucLength = 2;
        cResult = DXL_Reg_Write(ucID, _pDataBuf[_ucDataBufCntRead].pucBuf, ucLen);
        break;

      // Action
      case CAN_DXL_ACTION:
        ucLength = 2;
        cResult = DXL_Action(ucID);
        break;

      // Factory Reset
      case CAN_DXL_FACTORY_RESET:
        ucLength = 2;
        cResult = DXL_FactoryReset(ucID, _pDataBuf[_ucDataBufCntRead].pucBuf);
        break;

      // Reboot
      case CAN_DXL_REBOOT:
        ucLength = 2;
        cResult = DXL_Reboot(ucID);
        break;

      // Clear
      case CAM_DXL_CLEAR:
        ucLength = 2;
        cResult = DXL_Clear(ucID, _pDataBuf[_ucDataBufCntRead].pucBuf);
        break;

      // Status
      // Printing Instruction Error message and breaking the switch statement when it’s not an Instruction packet.
      default:
#ifdef USING_DEBUG_SERIAL
        Serial.println("Instruction Error!!");
#endif
        break;
    }

    // Increasing buffer counts. 
    _ucDataBufCntRead = (++_ucDataBufCntRead) % RECV_BUFFER_SIZE;

   // Checking the communication error. 
    if (!(cResult < 0)) {

      // Storing the Status Packet  error to the CAN data.. 
      // If the Instruction field has value indicating the Status, Instruction Error will be stored. 
      pucStatusDataBuffer[1] = (ucDxlInstruction == CAN_DXL_STATUS) ? 0x02 : dxl.getLastStatusPacketError();
      // Transmitting Status data. 
      mcp2515.transmit(ucID, pucStatusDataBuffer, ucLength);
    }

#ifdef USING_DEBUG_SERIAL
    // Printing ID, Instruction and Result in the serial monitor. 
    // Transmitting the ID, Ins
    Serial.print("ID          : "); Serial.println(ucID);
    Serial.print("Instruction : "); Serial.println(ucDxlInstruction);
    Serial.print("Result      : "); Serial.println(cResult);
#endif
  }
}


void spi_select() {

  digitalWrite(MKR_CAN_MCP2515_CS_PIN, LOW);
}



void spi_deselect() {

  digitalWrite(MKR_CAN_MCP2515_CS_PIN, HIGH);
}



uint8_t spi_transfer(uint8_t const ucData) {

  return SPI.transfer(ucData);
}



void onExternalEvent() {

  mcp2515.onExternalEventHandler();
}



void onReceiveBufferFull(uint32_t const uiTimestamp_us, uint32_t const uiID, uint8_t const * pucData, uint8_t const ucLen) {

  // Checking the buffer size
  // The occurrence condition : WriteCnt + 1 == ReadCnt
  if(((_ucDataBufCntWrite + 1) % RECV_BUFFER_SIZE) != _ucDataBufCntRead){

    // Storing data to the buffer. 
    _pDataBuf[_ucDataBufCntWrite].ucID = uiID;
    _pDataBuf[_ucDataBufCntWrite].ucLength = ucLen;
    memcpy(_pDataBuf[_ucDataBufCntWrite].pucBuf, pucData, ucLen);
  
    // Increase the buffer counts. 
    _ucDataBufCntWrite = (++_ucDataBufCntWrite) % RECV_BUFFER_SIZE;
  }
  else{
    Serial.println("Buffer is full!");
  }
}



int8_t DXL_Ping(uint8_t ucID, uint8_t *pucBuffer){

  int8_t cResult = -1;
  DYNAMIXEL::InfoFromPing_t modelInfo;
  modelInfo.model_number = 0;
  modelInfo.firmware_version = 0;
        
  cResult = dxl.ping(ucID, &modelInfo, 1, 10);

  pucBuffer[2] = (modelInfo.model_number & 0xFF);
  pucBuffer[3] = ((modelInfo.model_number >> 8) & 0xFF);
  pucBuffer[4] = modelInfo.firmware_version;

  return cResult;
}



int8_t DXL_Read(uint8_t ucID, uint8_t *pucInstructionBuffer, uint8_t* pucStatusBuffer){

  uint16_t usAddress = pucInstructionBuffer[1] | (pucInstructionBuffer[2] << 8);
  uint16_t usDataSize = pucInstructionBuffer[3];

  return dxl.read(ucID, usAddress, usDataSize, &pucStatusBuffer[2], 8);
}



int8_t DXL_Write(uint8_t ucID, uint8_t *pucInstructionBuffer, uint8_t ucLen){

  uint16_t usAddress = pucInstructionBuffer[1] | (pucInstructionBuffer[2] << 8);
  uint16_t usDataSize = ucLen - 3;

  return dxl.write(ucID, usAddress, &pucInstructionBuffer[3], usDataSize);
}



int8_t DXL_Reg_Write(uint8_t ucID, uint8_t *pucInstructionBuffer, uint8_t ucLen){

  uint16_t usAddress = pucInstructionBuffer[1] | (pucInstructionBuffer[2] << 8);
  uint16_t usDataSize = ucLen - 3;

  return dxl.regWrite(ucID, usAddress, &pucInstructionBuffer[3], usDataSize);
}



int8_t DXL_Action(uint8_t ucID){

  return dxl.action(ucID);
}



int8_t DXL_FactoryReset(uint8_t ucID, uint8_t *pucInstructionBuffer){

  return dxl.factoryReset(ucID, pucInstructionBuffer[1]);
}



int8_t DXL_Reboot(uint8_t ucID){

  return dxl.reboot(ucID);
}



int8_t DXL_Clear(uint8_t ucID, uint8_t *pucInstructionBuffer){
  
  uint32_t uiClearExOption = 0x44584C22;
  return dxl.clear(ucID, pucInstructionBuffer[1], uiClearExOption);
}
