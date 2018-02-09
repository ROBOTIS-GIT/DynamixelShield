/*
 *  dxl_def.h
 *
 *  dynamixel define
 *
 *  Created on: 2016. 10. 21.
 *
 *  Authors: Hancheol Cho (Baram) 
 *           KyungWan Ki  (Kei)
 */

#ifndef DXL_DEF_H
#define DXL_DEF_H

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>


#if defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_LEONARDO_ETH) || defined(ARDUINO_AVR_INDUSTRIAL101)
#define DXL_PORT Serial1
#elif defined(ARDUINO_SAM_ZERO)
#define DXL_PORT Serial5
#elif defined(ARDUINO_OpenCR)
#define DXL_PORT Serial1
#else
#define DXL_PORT Serial  
#endif



#define DXL_ID_BROADCAST_ID       0xFE

#define DXL_BUF_LENGTH            156


#define DXL_INST_PING             0x01
#define DXL_INST_READ             0x02
#define DXL_INST_WRITE            0x03
#define DXL_INST_REG_WRITE        0x04
#define DXL_INST_ACTION           0x05
#define DXL_INST_FACTORY_RESET    0x06
#define DXL_INST_REBOOT           0x08
#define DXL_INST_STATUS           0x55
#define DXL_INST_SYNC_READ        0x82
#define DXL_INST_SYNC_WRITE       0x83
#define DXL_INST_BULK_READ        0x92
#define DXL_INST_BULK_WRITE       0x93


#define DXL_ERR_RESULT_FAIL       0x01
#define DXL_ERR_INST              0x02
#define DXL_ERR_CRC               0x03
#define DXL_ERR_DATA_RANGE        0x04
#define DXL_ERR_DATA_LENGTH       0x05
#define DXL_ERR_DATA_LIMIT        0x06
#define DXL_ERR_ACCESS            0x07




#define ERR_NONE                      0
#define ERR_MEMORY                    1
#define ERR_FULL                      2
#define ERR_EMPTY                     3
#define ERR_NULL                      4
#define ERR_INVAILD_INDEX             5
#define ERR_TIMEOUT                   255
#define ERR_DXL_ERROR                 400
#define ERR_DXL_ERROR_RESP            700
#define ERR_DXL_NOT_OPEN              1000
#define ERR_DXL_WRITE_BUFFER          1001
#define ERR_DXL_NOT_FOUND             1002


#if !defined(BOARD_OpenCR)
typedef uint32_t  err_code_t;
#endif

typedef union
{
  uint8_t  u8Data[4];
  uint16_t u16Data[2];
  uint32_t u32Data;

  int8_t   s8Data[4];
  int16_t  s16Data[2];
  int32_t  s32Data;

  uint8_t  u8D;
  uint16_t u16D;
  uint32_t u32D;

  int8_t   s8D;
  int16_t  s16D;
  int32_t  s32D;
} data_t;


#endif
