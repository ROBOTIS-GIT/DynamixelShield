// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Example Environment
//
// - DYNAMIXEL: X series (except XL-320)
//              ID = 1, Baudrate = 1000000bps, Protocol 2.0
// - Controller: Arduino MKR WAN 1300
//               DYNAMIXEL Shield for Arduino MKR
//               MKR CAN Shield
// - CAN Interface: IXXAT USB-to-CAN V2
// - CAN Library: 107-Arduino-MCP2515
// - Software: IXXAT canAnalyser3 Mini
//             Bitrate = 1000CiA (1000 kbit/s)
//   + Transmit input examples(DYNAMIXEL ID 1):
//     [Ping] ID (hex) = 1, Data (hex) = 01
//     [Write LED ON] ID (hex) = 1, Data (hex) = 03 41 00 01
//     [Write LED OFF] ID (hex) = 1, Data (hex) = 03 41 00 00
//     [Write Torque ON] ID (hex) = 1, Data (hex) = 03 40 00 01
//     [Write Goal Position=512] ID (hex) = 1, Data (hex) = 03 74 00 00 02 00 00
//     [Read Present Position] ID (hex) = 1, Data (hex) = 03 84 00 04
// - https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/#examples
//
// Author: Seungmin Lee

#include <SPI.h>
#include <ArduinoMCP2515.h>
#include <DynamixelShield.h>

#undef max
#undef min
#include <algorithm>

// A data structure for the received CAN packet
typedef struct
{
  uint8_t dynamixel_id;
  uint8_t packet_length;
  uint8_t packet_buffer[8];
} CanData;

// Definitions for the instruction field value in the packet.
enum
{
  CAN_DXL_PING = 0x01,
  CAN_DXL_READ,
  CAN_DXL_WRITE,
  CAN_DXL_REG_WRITE,
  CAN_DXL_ACTION,
  CAN_DXL_FACTORY_RESET,
  CAN_DXL_REBOOT = 0x08,
  CAM_DXL_CLEAR = 0x10,
  CAN_DXL_STATUS = 0x55
};

// Function prototypes for MCP2515 control
void spi_select();
void spi_deselect();
uint8_t spi_transfer(uint8_t const);
void on_external_event();
void on_receive_buffer_full(uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

// User defined variables section
// [WARNING] When using "USING_DEBUG_SERIAL" below, set the CAN TX/RX cycle slower than 5ms.
// Setting faster than 5ms may stall the system.

// #define USING_DEBUG_SERIAL

#ifdef USING_DEBUG_SERIAL
const uint32_t SERIAL_BUADRATE = 9600;
#endif

// Define the SPI CS pin.
const uint8_t MKR_CAN_MCP2515_CS_PIN = 3;
// Define the external interrupt pin for RX event.
const uint8_t MKR_CAN_MCP2515_INT_PIN = 7;
// Define the RX buffer size.
const uint8_t RX_BUFFER_SIZE = 30;
// Communication Speed for CAN
// CanBitRate::BR_125kBPS_16MHZ
// CanBitRate::BR_250kBPS_16MHZ
// CanBitRate::BR_500kBPS_16MHZ
// CanBitRate::BR_1000kBPS_16MHZ
const CanBitRate CAN_BUADRATE = CanBitRate::BR_1000kBPS_16MHZ;

// Define the DYNAMIXEL Protocol version to use.
const float DXL_PROTOCOL_VERSION = 2.0;
// Define the Baud Rate of DYNAMIXEL to use. Recommend not to exceed 1Mbps.
const uint32_t DXL_BUADRATE = 1000000;

// Main code section
DynamixelShield dxl;
ArduinoMCP2515 mcp2515(
  spi_select,
  spi_deselect,
  spi_transfer,
  micros,
  on_receive_buffer_full,
  nullptr);

// Buffer to save the received data
CanData CanDataBuffer[RX_BUFFER_SIZE];
uint8_t write_buffer_index;
uint8_t read_buffer_index;

void setup()
{
  // Set the communication speed. Must correspond to the DYNAMIXEL’s Baud Rate in use.
  dxl.begin(DXL_BUADRATE);
  // Set the Protocol version. Must correspond to the DYNAMIXEL's Protocol Version in use.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize SPI and related pins.
  SPI.begin();
  pinMode(MKR_CAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKR_CAN_MCP2515_CS_PIN, HIGH);

  // Configure the external interrupt for CAN RX data.
  pinMode(MKR_CAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKR_CAN_MCP2515_INT_PIN), on_external_event, FALLING);

  // Initialize the CAN Control IC
  mcp2515.begin();
  mcp2515.setBitRate(CAN_BUADRATE);
  mcp2515.setNormalMode();

#ifdef USING_DEBUG_SERIAL
  // Initialize the USB serial port.
  Serial.begin(SERIAL_BUADRATE);
  Serial.println("Start CAN2DYNAMIXEL");
#endif
}

void loop()
{
  uint8_t id = 0;
  uint8_t length = 0;
  uint8_t instruction = 0;
  uint8_t status_packet_buffer[8] = {0x55, 0, };
  uint8_t status_packet_length = 0;
  int8_t packet_process_result = 0;

  // Run until the buffer is empty.
  for ( ; read_buffer_index != write_buffer_index; ) {
    // Get data from the receive buffer.
    id = CanDataBuffer[read_buffer_index].dynamixel_id;
    length = CanDataBuffer[read_buffer_index].packet_length;
    instruction = CanDataBuffer[read_buffer_index].packet_buffer[0];

    // Perform the instruction in the packet
    switch (instruction) {
      // Ping
      case CAN_DXL_PING:
        status_packet_length = 5;
        packet_process_result = DXL_Ping(id, status_packet_buffer);
        break;

      // Read
      case CAN_DXL_READ:
        status_packet_length = CanDataBuffer[read_buffer_index].packet_buffer[3] + 2;
        packet_process_result = DXL_Read(
          id,
          CanDataBuffer[read_buffer_index].packet_buffer,
          status_packet_buffer
        );
        break;

      // Write
      case CAN_DXL_WRITE:
        status_packet_length = 2;
        packet_process_result = DXL_Write(
          id,
          CanDataBuffer[read_buffer_index].packet_buffer,
          length
        );
        break;

      // Reg Write
      case CAN_DXL_REG_WRITE:
        status_packet_length = 2;
        packet_process_result = DXL_Reg_Write(
          id,
          CanDataBuffer[read_buffer_index].packet_buffer,
          length
        );
        break;

      // Action
      case CAN_DXL_ACTION:
        status_packet_length = 2;
        packet_process_result = DXL_Action(id);
        break;

      // Factory Reset
      case CAN_DXL_FACTORY_RESET:
        status_packet_length = 2;
        packet_process_result = DXL_FactoryReset(
          id,
          CanDataBuffer[read_buffer_index].packet_buffer
        );
        break;

      // Reboot
      case CAN_DXL_REBOOT:
        status_packet_length = 2;
        packet_process_result = DXL_Reboot(id);
        break;

      // Clear
      case CAM_DXL_CLEAR:
        status_packet_length = 2;
        packet_process_result = DXL_Clear(
          id,
          CanDataBuffer[read_buffer_index].packet_buffer
        );
        break;

      // Status
      // Print Instruction Error message and exit when Instruction type is not detected
      default:
#ifdef USING_DEBUG_SERIAL
        Serial.println("Instruction Error!!");
#endif
        break;
    }

    // Increase the read buffer index
    read_buffer_index++;
    read_buffer_index = read_buffer_index % RX_BUFFER_SIZE;

    // Check communication error
    if (packet_process_result > 0) {
      // Save the instruction error status in the Status packet.
      status_packet_buffer[1] =
        (instruction == CAN_DXL_STATUS) ? 0x02 : dxl.getLastStatusPacketError();
      // Transmit the Status packet.
      mcp2515.transmit(id, status_packet_buffer, status_packet_length);
    }

#ifdef USING_DEBUG_SERIAL
    Serial.print("ID          : "); Serial.println(id);
    Serial.print("Instruction : "); Serial.println(instruction);
    Serial.print("Result      : "); Serial.println(packet_process_result);
#endif
  }
}

void spi_select()
{
  digitalWrite(MKR_CAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  digitalWrite(MKR_CAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const ucData)
{
  return SPI.transfer(ucData);
}

void on_external_event()
{
  mcp2515.onExternalEventHandler();
}

void on_receive_buffer_full(
  uint32_t const uiTimestamp_us,
  uint32_t const uiID,
  uint8_t const * pucData,
  uint8_t const length)
{
  // Checking the buffer size
  // The occurrence condition : WriteCnt + 1 == ReadCnt
  if (((write_buffer_index + 1) % RX_BUFFER_SIZE) != read_buffer_index) {
    // Storing data to the buffer.
    CanDataBuffer[write_buffer_index].dynamixel_id = uiID;
    CanDataBuffer[write_buffer_index].packet_length = length;
    memcpy(CanDataBuffer[write_buffer_index].packet_buffer, pucData, length);

    // Increase the buffer counts.
    write_buffer_index++;
    write_buffer_index = write_buffer_index % RX_BUFFER_SIZE;
  } else {
    Serial.println("Buffer is full!");
  }
}

int8_t DXL_Ping(uint8_t id, uint8_t * pucBuffer)
{
  int8_t packet_process_result = -1;
  DYNAMIXEL::InfoFromPing_t modelInfo;
  modelInfo.model_number = 0;
  modelInfo.firmware_version = 0;

  packet_process_result = dxl.ping(id, &modelInfo, 1, 10);

  pucBuffer[2] = (modelInfo.model_number & 0xFF);
  pucBuffer[3] = ((modelInfo.model_number >> 8) & 0xFF);
  pucBuffer[4] = modelInfo.firmware_version;

  return packet_process_result;
}

int8_t DXL_Read(uint8_t id, uint8_t * pucInstructionBuffer, uint8_t * pucStatusBuffer)
{
  uint16_t control_table_address = pucInstructionBuffer[1] | (pucInstructionBuffer[2] << 8);
  uint16_t data_length = pucInstructionBuffer[3];

  return dxl.read(id, control_table_address, data_length, &pucStatusBuffer[2], 8);
}

int8_t DXL_Write(uint8_t id, uint8_t * pucInstructionBuffer, uint8_t length)
{
  uint16_t control_table_address = pucInstructionBuffer[1] | (pucInstructionBuffer[2] << 8);
  uint16_t data_length = length - 3;

  return dxl.write(id, control_table_address, &pucInstructionBuffer[3], data_length);
}

int8_t DXL_Reg_Write(uint8_t id, uint8_t * pucInstructionBuffer, uint8_t length)
{
  uint16_t control_table_address = pucInstructionBuffer[1] | (pucInstructionBuffer[2] << 8);
  uint16_t data_length = length - 3;

  return dxl.regWrite(id, control_table_address, &pucInstructionBuffer[3], data_length);
}

int8_t DXL_Action(uint8_t id)
{
  return dxl.action(id);
}

int8_t DXL_FactoryReset(uint8_t id, uint8_t * pucInstructionBuffer)
{
  return dxl.factoryReset(id, pucInstructionBuffer[1]);
}

int8_t DXL_Reboot(uint8_t id)
{
  return dxl.reboot(id);
}

int8_t DXL_Clear(uint8_t id, uint8_t * pucInstructionBuffer)
{
  uint32_t uiClearExOption = 0x44584C22;
  return dxl.clear(id, pucInstructionBuffer[1], uiClearExOption);
}
