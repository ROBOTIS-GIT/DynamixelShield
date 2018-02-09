/*
 * dxl.c
 *
 *  Created on: 2017. 4. 11.
 * 
 *  Authors: Hancheol Cho (Baram) 
 *           KyungWan Ki  (Kei)
 */
#include <stdlib.h>

#include "dxl_def.h"
#include "dxl.h"


#define PACKET_STATE_IDLE           0
#define PACKET_STATE_RESERVED       1
#define PACKET_STATE_ID             2
#define PACKET_STATE_LENGTH_L       3
#define PACKET_STATE_LENGTH_H       4
#define PACKET_STATE_DATA           5
#define PACKET_STATE_CRC_L          6
#define PACKET_STATE_CRC_H          7


#define PACKET_1_0_STATE_IDLE       0
#define PACKET_1_0_STATE_ID         1
#define PACKET_1_0_STATE_LENGTH     2
#define PACKET_1_0_STATE_DATA       3
#define PACKET_1_0_STATE_CHECK_SUM  4


#define DXL_DIR_PIN_RX              0
#define DXL_DIR_PIN_TX              1

//-- Internal Variables
//


//-- External Variables
//


//-- Internal Functions
//
static dxl_error_t dxlRxPacket1_0(dxl_t *p_packet, uint8_t data_in);
static dxl_error_t dxlMakePacketStatus1_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );
static dxl_error_t dxlTxPacketInst1_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length );


static dxl_error_t dxlRxPacket2_0(dxl_t *p_packet, uint8_t data_in);
static dxl_error_t dxlTxPacketInst2_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length );
static dxl_error_t dxlMakePacketStatus2_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );

static void dxlUpdateCrc(uint16_t *p_crc_cur, uint8_t data_in);
static uint16_t dxlAddStuffing(dxl_t *p_packet, uint8_t *p_data, uint16_t length);
static uint16_t dxlRemoveStuffing(uint8_t *p_data, uint16_t length);

static dxl_error_t dxlMakePacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );

static dxl_error_t dxlInstPing(dxl_t *p_dxl);
static dxl_error_t dxlInstFactoryReset(dxl_t *p_dxl);
static dxl_error_t dxlInstReboot(dxl_t *p_dxl);
static dxl_error_t dxlInstRead(dxl_t *p_dxl);
static dxl_error_t dxlInstWrite(dxl_t *p_dxl);
static dxl_error_t dxlInstSyncRead(dxl_t *p_dxl);
static dxl_error_t dxlInstSyncWrite(dxl_t *p_dxl);
static dxl_error_t dxlInstBulkRead(dxl_t *p_dxl);
static dxl_error_t dxlInstBulkWrite(dxl_t *p_dxl);

static dxl_error_t dxlInstPing1_0(dxl_t *p_dxl);
static dxl_error_t dxlInstPing2_0(dxl_t *p_dxl);

static dxl_error_t dxlInstRead1_0(dxl_t *p_dxl);
static dxl_error_t dxlInstRead2_0(dxl_t *p_dxl);

static dxl_error_t dxlInstWrite1_0(dxl_t *p_dxl);
static dxl_error_t dxlInstWrite2_0(dxl_t *p_dxl);

static dxl_error_t dxlInstSyncWrite1_0(dxl_t *p_dxl);
static dxl_error_t dxlInstSyncWrite2_0(dxl_t *p_dxl);

//-- External Functions
//





bool dxlInit(dxl_t *p_packet, uint8_t protocol_ver)
{
  p_packet->header_cnt = 0;
  p_packet->packet_ver = protocol_ver;
  p_packet->dxl_mode   = DXL_MODE_MASTER;

  p_packet->dxlport_ch = -1;
  p_packet->is_open    = false;
  p_packet->rx_state   = PACKET_STATE_IDLE;

  p_packet->process_state = DXL_PROCESS_INST;

  p_packet->id         = 200;

  p_packet->rx_timeout = 500*1000;
  p_packet->tx_timeout = 500*1000;

  p_packet->rx.header[0] = 0;
  p_packet->rx.header[1] = 0;
  p_packet->rx.header[2] = 0;


  p_packet->read_hook_func = NULL;


  p_packet->bypass_enable = false;
  p_packet->bypass_mode = DXL_BYPASS_NONE;

  p_packet->dir_pin = -1;
  p_packet->p_serial = NULL;

  return true;
}

bool dxlSetId(dxl_t *p_packet, uint8_t id)
{
  p_packet->id = id;

  return true;
}

uint8_t dxlGetId(dxl_t *p_packet)
{
  return p_packet->id;
}

bool dxlSetProtocolVersion(dxl_t *p_packet, uint8_t protocol_version)
{
  bool ret = true;


  if (protocol_version == DXL_PACKET_VER_1_0)
  {
    p_packet->packet_ver = protocol_version;
  }
  else if (protocol_version == DXL_PACKET_VER_2_0)
  {
    p_packet->packet_ver = protocol_version;
  }
  else
  {
    ret = false;
  }

  return ret;
}

uint8_t dxlGetProtocolVersion(dxl_t *p_packet)
{
  return p_packet->packet_ver;
}

void dxlSetDirPin(dxl_t *p_packet, int8_t dir_pin)
{
  p_packet->dir_pin = dir_pin;
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, DXL_DIR_PIN_RX);
}

void dxlSetSerial(dxl_t *p_packet, HardwareSerial *p_serial)
{
  p_packet->p_serial = p_serial;
}

bool dxlOpenPort(dxl_t *p_packet, uint32_t baud)
{
  bool ret = true;
  
  p_packet->dxlport_baud = baud;  
  if (p_packet->p_serial != NULL)
  {
    p_packet->p_serial->begin(baud);
    p_packet->is_open = true;
  }

  return ret;
}

bool dxlClosePort(dxl_t *p_packet)
{
  bool ret = true;

  return ret;
}

bool dxlIsOpen(dxl_t *p_packet)
{
  return p_packet->is_open;
}

void dxlSetRxByteISR(dxl_t *p_packet, void (*p_func)(uint8_t data))
{
  p_packet->read_hook_func = p_func;
}

uint32_t dxlRxAvailable(dxl_t *p_packet)
{
  return p_packet->p_serial->available();
}

uint8_t dxlRxRead(dxl_t *p_packet)
{
  return p_packet->p_serial->read();
}

void dxlTxWrite(dxl_t *p_packet, uint8_t *p_data, uint32_t length)
{
  uint32_t i;
  uint32_t pre_time;

  /*
  while(p_packet->p_serial->available())
  {
    p_packet->p_serial->read();
  }
  */

  digitalWrite(p_packet->dir_pin, DXL_DIR_PIN_TX);
  for (i=0; i<length; i++)
  {
    p_packet->p_serial->write(p_data[i]);
  }  

  p_packet->p_serial->flush();
  /*
  pre_time = millis();
  while(1)
  {
    if (p_packet->p_serial->availableForWrite() == (SERIAL_TX_BUFFER_SIZE - 1))
    {
      break;
    }
    if (millis()-pre_time >= 100)
    {
      break;
    }
  }  
  if (p_packet->dxlport_baud < 1000000)
  {
    delayMicroseconds(350);
  }
  else
  {
    delayMicroseconds(20);
  }
  */
  digitalWrite(p_packet->dir_pin, DXL_DIR_PIN_RX);
}

dxl_error_t dxlRxPacket(dxl_t *p_packet)
{
  uint8_t data;
  dxl_error_t ret = DXL_RET_EMPTY;


  while (dxlRxAvailable(p_packet))
  {
    data = dxlRxRead(p_packet);
    ret  = dxlRxPacketDataIn(p_packet, data);

    if (p_packet->read_hook_func != NULL)
    {
      (*p_packet->read_hook_func)(data);
    }
    if (ret != DXL_RET_EMPTY)
    {
      break;
    }
  }

  return ret;
}

dxl_error_t dxlRxPacketDataIn(dxl_t *p_packet, uint8_t data_in)
{
  dxl_error_t ret = DXL_RET_EMPTY;



  if (p_packet->packet_ver == DXL_PACKET_VER_1_0)
  {
    ret = dxlRxPacket1_0(p_packet, data_in);
  }
  else
  {
    ret  = dxlRxPacket2_0(p_packet, data_in);
  }

  return ret;
}

dxl_error_t dxlRxPacket1_0(dxl_t *p_packet, uint8_t data_in)
{
  dxl_error_t ret = DXL_RET_EMPTY;



  //-- time out
  //
  if( (micros() - p_packet->prev_time) > p_packet->rx_timeout )
  {
    p_packet->rx_state   = PACKET_1_0_STATE_IDLE;
    p_packet->prev_time  = micros();
    p_packet->header_cnt = 0;
  }
  p_packet->prev_time  = micros();


  switch(p_packet->rx_state)
  {
    case PACKET_1_0_STATE_IDLE:

      if( p_packet->header_cnt >= 1 )
      {
        p_packet->rx.header[1] = data_in;

        if(    p_packet->rx.header[0] == 0xFF
            && p_packet->rx.header[1] == 0xFF )
        {
          p_packet->header_cnt   = 0;
          p_packet->rx.check_sum = 0;
          p_packet->rx_state = PACKET_1_0_STATE_ID;
        }
        else
        {
          p_packet->rx.header[0] = p_packet->rx.header[1];
          p_packet->rx.header[1] = 0;
        }
      }
      else
      {
        p_packet->rx.header[p_packet->header_cnt] = data_in;
        p_packet->header_cnt++;
      }
      break;

    case PACKET_1_0_STATE_ID:
      p_packet->rx.id       = data_in;
      p_packet->rx_state    = PACKET_1_0_STATE_LENGTH;
      p_packet->rx.check_sum += data_in;
      break;

    case PACKET_1_0_STATE_LENGTH:
      p_packet->rx.packet_length = data_in;
      p_packet->rx_state         = PACKET_1_0_STATE_DATA;
      p_packet->rx.index         = 0;
      p_packet->rx.check_sum    += data_in;

      if (p_packet->rx.packet_length > DXL_MAX_BUFFER-4)
      {
        p_packet->rx_state = PACKET_1_0_STATE_IDLE;
        ret = DXL_RET_ERROR_LENGTH;
      }
      if (p_packet->rx.packet_length < 2)
      {
        p_packet->rx_state = PACKET_1_0_STATE_IDLE;
        ret = DXL_RET_ERROR_LENGTH;
      }
      break;

    case PACKET_1_0_STATE_DATA:
      p_packet->rx.data[p_packet->rx.index] = data_in;
      p_packet->rx.check_sum += data_in;

      p_packet->rx.index++;

      if (p_packet->rx.index >= p_packet->rx.packet_length-1)
      {
        p_packet->rx_state = PACKET_1_0_STATE_CHECK_SUM;
      }
      break;

    case PACKET_1_0_STATE_CHECK_SUM:
      p_packet->rx.check_sum          = ~(p_packet->rx.check_sum);
      p_packet->rx.check_sum_received = data_in;

      if (p_packet->rx.check_sum_received == p_packet->rx.check_sum)
      {
        p_packet->rx.cmd   = p_packet->rx.data[0];
        p_packet->rx.error = p_packet->rx.data[0];

        if (p_packet->dxl_mode == DXL_MODE_MASTER)
        {
          p_packet->rx.p_param      = &p_packet->rx.data[1];
          p_packet->rx.param_length = p_packet->rx.packet_length - 2;
          ret = DXL_RET_RX_STATUS;
        }
        else
        {
          p_packet->rx.p_param      = &p_packet->rx.data[1];
          p_packet->rx.param_length = p_packet->rx.packet_length - 2;
          ret = DXL_RET_RX_INST;
        }
      }
      else
      {
        ret = DXL_RET_ERROR_CHECK_SUM;
      }

      p_packet->rx_state = PACKET_1_0_STATE_IDLE;
      break;

    default:
      p_packet->rx_state = PACKET_1_0_STATE_IDLE;
      break;
  }

  return ret;
}

dxl_error_t dxlRxPacket2_0(dxl_t *p_packet, uint8_t data_in)
{
  dxl_error_t ret = DXL_RET_EMPTY;
  uint16_t stuff_length;



  //-- time out
  //
  if( (micros() - p_packet->prev_time) > p_packet->rx_timeout )
  {
    p_packet->rx_state   = PACKET_STATE_IDLE;
    p_packet->header_cnt = 0;
  }
  p_packet->prev_time  = micros();


  switch(p_packet->rx_state)
  {
    case PACKET_STATE_IDLE:
      if( p_packet->header_cnt >= 2 )
      {
        p_packet->rx.header[2] = data_in;

        if(    p_packet->rx.header[0] == 0xFF
            && p_packet->rx.header[1] == 0xFF
            && p_packet->rx.header[2] == 0xFD )
        {
          p_packet->header_cnt = 0;
          p_packet->rx.crc     = 0;
          dxlUpdateCrc(&p_packet->rx.crc, 0xFF);
          dxlUpdateCrc(&p_packet->rx.crc, 0xFF);
          dxlUpdateCrc(&p_packet->rx.crc, 0xFD);
          p_packet->rx_state = PACKET_STATE_RESERVED;
        }
        else
        {
          p_packet->rx.header[0] = p_packet->rx.header[1];
          p_packet->rx.header[1] = p_packet->rx.header[2];
          p_packet->rx.header[2] = 0;
        }
      }
      else
      {
        p_packet->rx.header[p_packet->header_cnt] = data_in;
        p_packet->header_cnt++;
      }
      break;

    case PACKET_STATE_RESERVED:
      if( data_in == 0xFD )
      {
        p_packet->rx_state  = PACKET_STATE_IDLE;
        ret = DXL_RET_ERROR_LENGTH;
      }
      else
      {
        p_packet->rx.reserved = data_in;
        p_packet->rx_state    = PACKET_STATE_ID;
      }
      dxlUpdateCrc(&p_packet->rx.crc, data_in);
      break;

    case PACKET_STATE_ID:
      p_packet->rx.id       = data_in;
      p_packet->rx_state    = PACKET_STATE_LENGTH_L;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);
      break;

    case PACKET_STATE_LENGTH_L:
      p_packet->rx.packet_length = data_in;
      p_packet->rx_state         = PACKET_STATE_LENGTH_H;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);
      break;

    case PACKET_STATE_LENGTH_H:
      p_packet->rx.packet_length |= data_in<<8;
      p_packet->rx_state          = PACKET_STATE_DATA;
      p_packet->rx.index          = 0;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);

      if (p_packet->rx.packet_length > DXL_MAX_BUFFER)
      {
        p_packet->rx_state = PACKET_STATE_IDLE;
        ret = DXL_RET_ERROR_LENGTH;
      }
      if (p_packet->rx.packet_length < 3)
      {
        p_packet->rx_state = PACKET_STATE_IDLE;
        ret = DXL_RET_ERROR_LENGTH;
      }
      break;

    case PACKET_STATE_DATA:
      p_packet->rx.data[p_packet->rx.index] = data_in;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);

      p_packet->rx.index++;

      if (p_packet->rx.index >= p_packet->rx.packet_length-2)
      {
        p_packet->rx_state = PACKET_STATE_CRC_L;
      }
      break;

    case PACKET_STATE_CRC_L:
      p_packet->rx.crc_received = data_in;
      p_packet->rx_state        = PACKET_STATE_CRC_H;
      break;

    case PACKET_STATE_CRC_H:
      p_packet->rx.crc_received |= data_in<<8;


      stuff_length = dxlRemoveStuffing(p_packet->rx.data, p_packet->rx.packet_length);
      p_packet->rx.packet_length -= stuff_length;

      if (p_packet->rx.crc_received == p_packet->rx.crc)
      {
        p_packet->rx.cmd   = p_packet->rx.data[0];
        p_packet->rx.error = p_packet->rx.data[1];

        if (p_packet->rx.data[0] == DXL_INST_STATUS)
        {
          p_packet->rx.p_param      = &p_packet->rx.data[2];
          p_packet->rx.param_length = p_packet->rx.packet_length - 4;
          ret = DXL_RET_RX_STATUS;
        }
        else
        {
          p_packet->rx.p_param      = &p_packet->rx.data[1];
          p_packet->rx.param_length = p_packet->rx.packet_length - 3;
          ret = DXL_RET_RX_INST;
        }
      }
      else
      {
        ret = DXL_RET_ERROR_CRC;
      }

      p_packet->rx_state = PACKET_STATE_IDLE;
      break;

    default:
      p_packet->rx_state = PACKET_STATE_IDLE;
      break;
  }

  return ret;
}

uint16_t dxlRemoveStuffing(uint8_t *p_data, uint16_t length)
{
  uint16_t i;
  uint16_t stuff_length;
  uint16_t index;

  index = 0;
  stuff_length = 0;
  for( i=0; i<length; i++ )
  {
    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        i++;
        stuff_length++;
      }
    }
    p_data[index++] = p_data[i];
  }

  return stuff_length;
}

uint16_t dxlAddStuffing(dxl_t *p_packet, uint8_t *p_data, uint16_t length)
{
  uint8_t *stuff_buf; //[DXL_MAX_BUFFER];
  uint16_t i;
  uint16_t index;
  uint16_t stuff_length = 0;


  stuff_buf = p_packet->rx.data;

  if (length <= 2)
  {
    return 0;
  }

  index = 0;
  stuff_length = 0;
  for( i=0; i<length-2; i++ )
  {
    stuff_buf[index++] = p_data[i];

    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        stuff_buf[index++] = 0xFD;
        stuff_length++;
      }
    }
  }

  if( stuff_length > 0 )
  {
    for( i=0; i<index; i++ )
    {
      p_data[i] = stuff_buf[i];
    }
  }

  return stuff_length;
}

dxl_error_t dxlTxPacket(dxl_t *p_packet)
{
  dxl_error_t ret = DXL_RET_OK;
  
  dxlTxWrite(p_packet, p_packet->tx.data, p_packet->tx.packet_length);

  return ret;
}

dxl_error_t dxlTxPacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  dxl_error_t ret = DXL_RET_EMPTY;


  ret = dxlMakePacketStatus(p_packet, id, error, p_data, length);
  if (ret == DXL_RET_OK)
  {
    dxlTxWrite(p_packet, p_packet->tx.data, p_packet->tx.packet_length);
  }

  return ret;
}

dxl_error_t dxlMakePacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  dxl_error_t ret = DXL_RET_EMPTY;


  if (p_packet->packet_ver == DXL_PACKET_VER_1_0)
  {
    ret  = dxlMakePacketStatus1_0(p_packet, id, error, p_data, length);
  }
  else
  {
    ret  = dxlMakePacketStatus2_0(p_packet, id, error, p_data, length);
  }

  return ret;
}

dxl_error_t dxlMakePacketStatus1_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint8_t  check_sum;


  if (length > DXL_MAX_BUFFER-6)
  {
    return DXL_RET_ERROR_LENGTH;
  }
  if (length > 255)
  {
    return DXL_RET_ERROR_LENGTH;
  }


  check_sum = 0;
  packet_length = length + 2; // param_length + Instruction + CheckSum

  p_packet->tx.data[PKT_1_0_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_ID_IDX]    = id;
  p_packet->tx.data[PKT_1_0_ERROR_IDX] = error;

  check_sum += id;
  check_sum += packet_length;
  check_sum += error;

  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_1_0_STATUS_PARAM_IDX + i] = p_data[i];
    check_sum += p_data[i];
  }

  p_packet->tx.data[PKT_1_0_LEN_IDX] = packet_length;
  p_packet->tx.data[PKT_1_0_ERROR_IDX + packet_length - 1] = ~(check_sum);

  // 데이터 전송
  dxlTxWrite(p_packet, p_packet->tx.data, packet_length + 4);

  return ret;
}

dxl_error_t dxlMakePacketStatus2_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint16_t stuff_length;
  uint16_t crc;


  if (length > DXL_MAX_BUFFER-7)
  {
    return DXL_RET_ERROR_LENGTH;
  }

  packet_length = length + 4; // param_length + Instruction + Error + CRC_L + CRC_H

  p_packet->tx.data[PKT_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_3_IDX] = 0xFD;
  p_packet->tx.data[PKT_RSV_IDX]   = 0x00;
  p_packet->tx.data[PKT_ID_IDX]    = id;
  p_packet->tx.data[PKT_INST_IDX]  = DXL_INST_STATUS;
  p_packet->tx.data[PKT_ERROR_IDX] = error;


  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_STATUS_PARAM_IDX + i] = p_data[i];
  }

  // stuff 추가
  stuff_length = dxlAddStuffing(p_packet, &p_packet->tx.data[PKT_INST_IDX], length + 2); // + instruction + error
  packet_length += stuff_length;

  p_packet->tx.data[PKT_LEN_L_IDX] = packet_length >> 0;
  p_packet->tx.data[PKT_LEN_H_IDX] = packet_length >> 8;


  // crc 계산
  crc = 0;
  for (i=0; i<packet_length+7-2; i++)
  {
    dxlUpdateCrc(&crc, p_packet->tx.data[i]);
  }


  p_packet->tx.data[PKT_INST_IDX + packet_length - 2] = crc >> 0;
  p_packet->tx.data[PKT_INST_IDX + packet_length - 1] = crc >> 8;

  p_packet->tx.packet_length = packet_length + 7;

  return ret;
}

dxl_error_t dxlTxPacketInst(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length )
{
  dxl_error_t ret = DXL_RET_EMPTY;


  if (p_packet->packet_ver == DXL_PACKET_VER_1_0)
  {
    ret = dxlTxPacketInst1_0(p_packet, id, inst_cmd, p_data, length);
  }
  else
  {
    ret  = dxlTxPacketInst2_0(p_packet, id, inst_cmd, p_data, length);
  }

  return ret;
}

dxl_error_t dxlTxPacketInst1_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length )
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint8_t  check_sum;


  if (length > DXL_MAX_BUFFER-6)
  {
    return DXL_RET_ERROR_LENGTH;
  }
  if (length > 255)
  {
    return DXL_RET_ERROR_LENGTH;
  }


  check_sum = 0;
  packet_length = length + 2; // param_length + Instruction + CheckSum

  p_packet->tx.data[PKT_1_0_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_ID_IDX]    = id;
  p_packet->tx.data[PKT_1_0_INST_IDX]  = inst_cmd;

  check_sum += id;
  check_sum += packet_length;
  check_sum += inst_cmd;

  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_1_0_INST_PARAM_IDX + i] = p_data[i];
    check_sum += p_data[i];
  }

  p_packet->tx.data[PKT_1_0_LEN_IDX] = packet_length;
  p_packet->tx.data[PKT_1_0_INST_IDX + packet_length - 1] = ~(check_sum);

  // 데이터 전송
  dxlTxWrite(p_packet, p_packet->tx.data, packet_length + 4);

  return ret;
}

dxl_error_t dxlTxPacketInst2_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length )
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint16_t stuff_length;
  uint16_t crc;


  if (length > DXL_MAX_BUFFER-7)
  {
    return DXL_RET_ERROR_LENGTH;
  }

  packet_length = length + 3; // param_length + Instruction + CRC_L + CRC_H

  p_packet->tx.data[PKT_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_3_IDX] = 0xFD;
  p_packet->tx.data[PKT_RSV_IDX]   = 0x00;
  p_packet->tx.data[PKT_ID_IDX]    = id;
  p_packet->tx.data[PKT_INST_IDX]  = inst_cmd;

  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_INST_PARAM_IDX + i] = p_data[i];
  }

  // stuff 추가
  stuff_length = dxlAddStuffing(p_packet, &p_packet->tx.data[PKT_INST_IDX], length + 1);  // + instruction
  packet_length += stuff_length;

  p_packet->tx.data[PKT_LEN_L_IDX] = packet_length >> 0;
  p_packet->tx.data[PKT_LEN_H_IDX] = packet_length >> 8;

  
  // crc 계산
  crc = 0;
  for (i=0; i<packet_length+7-2; i++)
  {
    dxlUpdateCrc(&crc, p_packet->tx.data[i]);
  }


  p_packet->tx.data[PKT_INST_IDX + packet_length - 2] = crc >> 0;
  p_packet->tx.data[PKT_INST_IDX + packet_length - 1] = crc >> 8;


  // 데이터 전송
  dxlTxWrite(p_packet, p_packet->tx.data, packet_length + 7);

  return ret;
}



#if 1
const unsigned short crc_table[256] PROGMEM  = {0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

void dxlUpdateCrc(uint16_t *p_crc_cur, uint8_t data_in)
{
uint16_t crc;
uint16_t i;

crc = *p_crc_cur;

i = ((unsigned short)(crc >> 8) ^ data_in) & 0xFF;
*p_crc_cur = (crc << 8) ^ pgm_read_word_near(crc_table + i);
}
#else

const unsigned short crc_table[256] = {0x0000,
                                0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                                0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                                0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                                0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                                0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                                0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                                0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                                0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                                0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                                0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                                0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                                0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                                0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                                0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                                0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                                0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                                0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                                0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                                0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                                0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                                0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                                0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                                0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                                0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                                0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                                0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                                0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                                0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                                0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                                0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                                0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                                0x820D, 0x8207, 0x0202 };

void dxlUpdateCrc(uint16_t *p_crc_cur, uint8_t data_in)
{
  uint16_t crc;
  uint16_t i;

  crc = *p_crc_cur;

  i = ((unsigned short)(crc >> 8) ^ data_in) & 0xFF;
  *p_crc_cur = (crc << 8) ^ crc_table[i];
}
#endif


