/*
 * dxl.h
 *
 *  Created on: 2017. 4. 11.
 *
 *  Authors: Hancheol Cho (Baram) 
 *           KyungWan Ki  (Kei)
 */

#ifndef DXL_H_
#define DXL_H_

#include <Arduino.h>
#include "dxl_def.h"



#ifdef __cplusplus
 extern "C" {
#endif


#define DXL_PACKET_VER_1_0      0
#define DXL_PACKET_VER_2_0      1


#define DXL_STATE_WAIT_INST     0
#define DXL_STATE_WAIT_STATUS   1


#define DXL_TYPE_INST           0
#define DXL_TYPE_STATUS         1

#define DXL_MODE_SLAVE          0
#define DXL_MODE_MASTER         1


#define DXL_INST_STATUS         0x55

#define DXL_GLOBAL_ID           0xFE
#define DXL_ALL_ID              0xFE



#define DXL_MAX_BUFFER          DXL_BUF_LENGTH


//-- 2.0 Protocol
//
#define PKT_HDR_1_IDX           0
#define PKT_HDR_2_IDX           1
#define PKT_HDR_3_IDX           2
#define PKT_RSV_IDX             3
#define PKT_ID_IDX              4
#define PKT_LEN_L_IDX           5
#define PKT_LEN_H_IDX           6
#define PKT_INST_IDX            7
#define PKT_ERROR_IDX           8

#define PKT_INST_PARAM_IDX      8
#define PKT_STATUS_PARAM_IDX    9


//-- 1.0 Protocol
//
#define PKT_1_0_HDR_1_IDX         0
#define PKT_1_0_HDR_2_IDX         1
#define PKT_1_0_ID_IDX            2
#define PKT_1_0_LEN_IDX           3
#define PKT_1_0_INST_IDX          4
#define PKT_1_0_ERROR_IDX         4

#define PKT_1_0_INST_PARAM_IDX    5
#define PKT_1_0_STATUS_PARAM_IDX  5



#define INST_PING               0x01
#define INST_READ               0x02
#define INST_WRITE              0x03
#define INST_REG_WRITE          0x04
#define INST_ACTION             0x05
#define INST_RESET              0x06
#define INST_REBOOT             0x08
#define INST_STATUS             0x55
#define INST_SYNC_READ          0x82
#define INST_SYNC_WRITE         0x83
#define INST_BULK_READ          0x92
#define INST_BULK_WRITE         0x93

#define INST_MEMORY_ERASE       0xF0
#define INST_MEMORY_READ        0xF1
#define INST_MEMORY_WRITE       0xF2


#define DXL_ERR_NONE            0x00
#define DXL_ERR_RESULT_FAIL     0x01
#define DXL_ERR_INST_ERROR      0x02
#define DXL_ERR_CRC_ERROR       0x03
#define DXL_ERR_DATA_RANGE      0x04
#define DXL_ERR_DATA_LENGTH     0x05
#define DXL_ERR_DATA_LIMIT      0x06
#define DXL_ERR_ACCESS          0x07


#define DXL_PROCESS_INST        0
#define DXL_PROCESS_BROAD_PING  1
#define DXL_PROCESS_BROAD_READ  2
#define DXL_PROCESS_BROAD_WRITE 3

#define DXL_BYPASS_NONE         0
#define DXL_BYPASS_ENABLE       1
#define DXL_BYPASS_ONLY         2



typedef enum
{
  DXL_RET_OK,
  DXL_RET_RX_INST,
  DXL_RET_RX_STATUS,
  DXL_RET_RX_RESP,
  DXL_RET_EMPTY,
  DXL_RET_ERROR_CRC,
  DXL_RET_ERROR_CHECK_SUM,
  DXL_RET_ERROR_LENGTH,
  DXL_RET_ERROR_RX_BUFFER,
  DXL_RET_ERROR_NO_ID,
  DXL_RET_ERROR_NOT_GLOBALID,
  DXL_RET_ERROR_NOT_COMMAND,
  DXL_RET_ERROR,
  DXL_RET_NOT_OPEN,
  DXL_RET_PROCESS_BROAD_PING,
  DXL_RET_PROCESS_BROAD_READ,
  DXL_RET_PROCESS_BROAD_WRITE
} dxl_error_t;




typedef struct
{
  uint8_t   header[3];
  uint8_t   reserved;
  uint8_t   id;
  uint8_t   cmd;
  uint8_t   error;
  uint8_t   type;
  uint16_t  index;
  uint16_t  packet_length;
  uint16_t  param_length;
  uint16_t  crc;
  uint16_t  crc_received;
  uint8_t   check_sum;
  uint8_t   check_sum_received;
  uint8_t   *p_param;
  uint32_t  dummy;
  uint8_t   data[DXL_MAX_BUFFER];
} dxl_packet_t;


typedef struct
{
  uint8_t  packet_ver;
  uint8_t  dxl_mode;
   int8_t  dxlport_ch;
  uint32_t dxlport_baud;
  
  int8_t   dir_pin;
  HardwareSerial *p_serial;

  bool     is_open;
  uint8_t  rx_state;
  uint8_t  id;
  uint8_t  current_id;
  uint8_t  pre_id;

  bool     bypass_enable;
  uint8_t  bypass_mode;
  uint8_t  bypass_dxlport_ch;

  uint8_t  process_state;
  uint32_t process_pre_time;


  uint32_t rx_timeout;
  uint32_t tx_timeout;
  uint32_t prev_time;
  uint32_t tx_time;
  uint32_t rx_time;
  uint8_t  header_cnt;

  void (*read_hook_func)(uint8_t data);

  dxl_packet_t    rx;
  dxl_packet_t    tx;
} dxl_t;




bool dxlInit(dxl_t *p_packet, uint8_t protocol_ver);
bool dxlOpenPort(dxl_t *p_packet, uint32_t baud);
bool dxlClosePort(dxl_t *p_packet);
bool dxlIsOpen(dxl_t *p_packet);
void dxlSetRxByteISR(dxl_t *p_packet, void (*p_func)(uint8_t data));

void dxlSetDirPin(dxl_t *p_packet, int8_t dir_pin);
void dxlSetSerial(dxl_t *p_packet, HardwareSerial *p_serial);

bool    dxlSetId(dxl_t *p_packet, uint8_t id);
uint8_t dxlGetId(dxl_t *p_packet);

bool    dxlSetProtocolVersion(dxl_t *p_packet, uint8_t protocol_version);
uint8_t dxlGetProtocolVersion(dxl_t *p_packet);

uint32_t dxlRxAvailable(dxl_t *p_packet);
uint8_t  dxlRxRead(dxl_t *p_packet);


dxl_error_t dxlRxPacket(dxl_t *p_packet);
dxl_error_t dxlRxPacketDataIn(dxl_t *p_packet, uint8_t data_in);

dxl_error_t dxlTxPacket(dxl_t *p_packet);
dxl_error_t dxlTxPacketInst(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length);
dxl_error_t dxlTxPacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length);

#ifdef __cplusplus
}
#endif



#endif /* DXL_H_ */
