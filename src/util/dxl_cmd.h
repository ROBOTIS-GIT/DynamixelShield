/*
 *  dxl_hw.h
 *
 *  dynamixel hardware
 *
 *  Created on: 2016. 10. 21.
 *  
 *  Authors: Hancheol Cho (Baram) 
 *           KyungWan Ki  (Kei)
 */

#ifndef DXL_CMD_H
#define DXL_CMD_H


#include "dxl_def.h"


#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
}
#endif



#define DXLCMD_MAX_NODE               16
#define DXLCMD_MAX_NODE_BUFFER_SIZE   8
#define DXLCMD_MAX_BUFFER             DXL_BUF_LENGTH






typedef struct
{
  uint8_t id;
  uint8_t error;
} dxlcmd_resp_reset_t;

typedef struct
{
  uint8_t id;
  uint8_t error;
} dxlcmd_resp_reboot_t;

typedef struct
{
  uint8_t   id;
  uint8_t   error;
  uint16_t  model_number;
  uint8_t   firmware_version;
} dxlcmd_ping_node_t;

typedef struct
{
  uint8_t               id_count;
  dxlcmd_ping_node_t   *p_node[DXLCMD_MAX_NODE];
  uint32_t              mem[DXLCMD_MAX_BUFFER/4];
} dxlcmd_resp_ping_t;


typedef struct
{
  uint8_t  id;
  uint8_t  error;
  uint16_t length;
  uint8_t  *p_data;
} dxlcmd_read_node_t;

typedef struct
{
  uint8_t               id_count;
  dxlcmd_read_node_t   *p_node[DXLCMD_MAX_NODE];
  uint32_t              mem[DXLCMD_MAX_BUFFER/4];
} dxlcmd_resp_read_t;

typedef struct
{
  uint8_t id;
  uint8_t error;
} dxlcmd_resp_write_t;

//-- SyncRead
//
typedef struct
{
  uint8_t   id_count;
  uint8_t   id_tbl[DXLCMD_MAX_NODE];
  uint16_t  addr;
  uint8_t   length;
} dxlcmd_param_sync_read_t;


//-- BulkRead
//
typedef struct
{
  uint8_t   id_count;
  uint8_t   id_tbl[DXLCMD_MAX_NODE];
  uint16_t  addr  [DXLCMD_MAX_NODE];
  uint8_t   length[DXLCMD_MAX_NODE];
} dxlcmd_param_bulk_read_t;


//-- SyncWrite
//
typedef struct
{
  uint8_t  id;
  uint8_t  data[DXLCMD_MAX_NODE_BUFFER_SIZE];
} dxlcmd_sync_write_node_t;

typedef struct
{
  uint8_t  id_count;
  uint16_t addr;
  uint8_t  length;
  dxlcmd_sync_write_node_t node[DXLCMD_MAX_NODE];
} dxlcmd_param_sync_write_t;


//-- BulkWrite
//
typedef struct
{
  uint8_t  id;
  uint16_t addr;
  uint8_t  length;
  uint8_t  data[DXLCMD_MAX_NODE_BUFFER_SIZE];
} dxlcmd_bulk_write_node_t;

typedef struct
{
  uint8_t   id_count;
  dxlcmd_bulk_write_node_t node[DXLCMD_MAX_NODE];
} dxlcmd_param_bulk_write_t;



typedef union
{
  dxlcmd_param_sync_read_t  sync_read;
  dxlcmd_param_bulk_read_t  bulk_read;
  dxlcmd_param_sync_write_t sync_write;
  dxlcmd_param_bulk_write_t bulk_write;
} dxlcmd_param_t;

typedef union
{
  dxlcmd_resp_reset_t   reset;
  dxlcmd_resp_reboot_t  reboot;
  dxlcmd_resp_ping_t    ping;

  dxlcmd_resp_read_t    read;
  dxlcmd_resp_write_t   write;

  dxlcmd_resp_read_t    sync_read;
  dxlcmd_resp_read_t    bulk_read;
} dxlcmd_resp_t;


typedef union
{
  dxlcmd_param_t  param;
  dxlcmd_resp_t   resp;
} dxlcmd_param_resp_t;


// TODO : 향후에 다른곳으로 이동이 필요함
data_t getDataType(uint8_t *p_data, uint8_t length);


bool dxlcmdInit(void);


dxl_error_t dxlcmdReset(dxl_t *p_packet, uint8_t id, uint8_t option, dxlcmd_resp_reset_t *p_resp, uint32_t timeout);
dxl_error_t dxlcmdReboot(dxl_t *p_packet, uint8_t id, dxlcmd_resp_reboot_t *p_resp, uint32_t timeout);

dxl_error_t dxlcmdPing(dxl_t *p_packet, uint8_t id, dxlcmd_resp_ping_t *p_resp, uint32_t timeout);

dxl_error_t dxlcmdRead(dxl_t *p_packet, uint8_t id, uint16_t addr, uint16_t length, dxlcmd_resp_read_t *p_resp, uint32_t timeout);
dxl_error_t dxlcmdWrite(dxl_t *p_packet, uint8_t id, uint8_t *p_data, uint16_t addr, uint16_t length, dxlcmd_resp_write_t *p_resp, uint32_t timeout);
dxl_error_t dxlcmdWriteNoResp(dxl_t *p_packet, uint8_t id, uint8_t *p_data, uint16_t addr, uint16_t length);

dxl_error_t dxlcmdSyncRead(dxl_t *p_packet, dxlcmd_param_sync_read_t *p_param, dxlcmd_resp_read_t *p_resp, uint32_t timeout);
dxl_error_t dxlcmdBulkRead(dxl_t *p_packet, dxlcmd_param_bulk_read_t *p_param, dxlcmd_resp_read_t *p_resp, uint32_t timeout);

dxl_error_t dxlcmdSyncWrite(dxl_t *p_packet, dxlcmd_param_sync_write_t *p_param);
dxl_error_t dxlcmdBulkWrite(dxl_t *p_packet, dxlcmd_param_bulk_write_t *p_param);



#endif
