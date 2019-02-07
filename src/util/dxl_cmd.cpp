/*
 *  dxl_cmd.cpp
 *
 *  dynamixel hardware
 *
 *  Created on: 2016. 10. 21.
 *
 *  Authors: Hancheol Cho (Baram) 
 *           KyungWan Ki  (Kei)
 */

#include "dxl.h" 
#include "dxl_cmd.h"







//-- Internal Variables
//



//-- External Variables
//


//-- Internal Functions
//


//-- External Functions
//




bool dxlcmdInit(void)
{
  bool ret = true;
  
  return ret;
}

dxl_error_t dxlcmdReset(dxl_t *p_packet, uint8_t id, uint8_t option, dxlcmd_resp_reset_t *p_resp, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;
  uint8_t tx_param[1];


  if (id == DXL_GLOBAL_ID)
  {
    return DXL_RET_ERROR_NOT_GLOBALID;
  }

  if (dxlIsOpen(p_packet) == true)
  {
    tx_param[0] = option;

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_RESET, tx_param, 1);
    p_packet->tx_time = micros() - pre_time_us;

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;

        p_resp->id    = p_packet->rx.id;
        p_resp->error = p_packet->rx.error;

        dxl_ret = DXL_RET_RX_RESP;
        break;
      }
      else if (dxl_ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdReboot(dxl_t *p_packet, uint8_t id, dxlcmd_resp_reboot_t *p_resp, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;


  if (id == DXL_GLOBAL_ID)
  {
    return DXL_RET_ERROR_NOT_GLOBALID;
  }

  if (dxlIsOpen(p_packet) == true)
  {
    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_REBOOT, NULL, 0);
    p_packet->tx_time = micros() - pre_time_us;

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;

        p_resp->id    = p_packet->rx.id;
        p_resp->error = p_packet->rx.error;

        dxl_ret = DXL_RET_RX_RESP;
        break;
      }
      else if (dxl_ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdPing(dxl_t *p_packet, uint8_t id, dxlcmd_resp_ping_t *p_resp, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_ms;
  uint32_t pre_time_us;
  uint32_t mem_addr;
  uint8_t  *p_mem = (uint8_t *)p_resp->mem;


  p_resp->id_count = 0;
  p_resp->p_node[0] = (dxlcmd_ping_node_t *)&p_mem[0];


  if (dxlIsOpen(p_packet) == true)
  {
    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_PING, NULL, 0);
    p_packet->tx_time = micros() - pre_time_us;
    

    mem_addr = 0;
    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS && p_resp->id_count < DXLCMD_MAX_NODE)
      {
        p_packet->rx_time = micros() - pre_time_us;
        pre_time_ms       = millis();

        p_resp->p_node[p_resp->id_count]->id = p_packet->rx.id;
        p_resp->p_node[p_resp->id_count]->model_number     = p_packet->rx.p_param[0]<<0;
        p_resp->p_node[p_resp->id_count]->model_number    |= p_packet->rx.p_param[1]<<8;
        p_resp->p_node[p_resp->id_count]->firmware_version = p_packet->rx.p_param[2];

        p_resp->id_count++;

        //-- 주소를 4바이트로 정렬( 구조체를 직접 타입변환하여 사용하기 위해서 )
        //
        mem_addr += sizeof(dxlcmd_ping_node_t);
        if (mem_addr%4)
        {
          mem_addr += 4 - (mem_addr%4);
        }

        p_resp->p_node[p_resp->id_count] = (dxlcmd_ping_node_t *)&p_mem[mem_addr];

        if (id != DXL_GLOBAL_ID)
        {
          dxl_ret = DXL_RET_RX_RESP;
          break;
        }
      }

      if (millis()-pre_time_ms >= timeout)
      {
        if (p_resp->id_count > 0)
        {
          dxl_ret = DXL_RET_RX_RESP;
        }
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}


dxl_error_t dxlcmdRead(dxl_t *p_packet, uint8_t id, uint16_t addr, uint16_t length, dxlcmd_resp_read_t *p_resp, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[4];
  uint32_t mem_addr;
  uint8_t  *p_mem = (uint8_t *)p_resp->mem;
  uint32_t i;


  p_resp->id_count = 0;
  p_resp->p_node[0] = (dxlcmd_read_node_t *)&p_mem[0];


  if (id == DXL_GLOBAL_ID)
  {
    return DXL_RET_ERROR_NOT_GLOBALID;
  }

  if (dxlIsOpen(p_packet) == true)
  {
    if (p_packet->packet_ver == DXL_PACKET_VER_1_0 )
    {
      tx_param[0] = addr;
      tx_param[1] = length;
    }
    else
    {
      tx_param[0] = addr >> 0;
      tx_param[1] = addr >> 8;
      tx_param[2] = length >> 0;
      tx_param[3] = length >> 8;
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_READ, tx_param, 4);
    p_packet->tx_time = micros() - pre_time_us;

    mem_addr = 0;
    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;

        mem_addr += sizeof(dxlcmd_read_node_t);
        p_resp->p_node[p_resp->id_count]->p_data = &p_mem[mem_addr];

        p_resp->p_node[p_resp->id_count]->id     = p_packet->rx.id;
        p_resp->p_node[p_resp->id_count]->error  = p_packet->rx.error;
        p_resp->p_node[p_resp->id_count]->length = p_packet->rx.param_length;

        if ((mem_addr+p_packet->rx.param_length) > DXLCMD_MAX_BUFFER)
        {
          break;
        }

        for (i=0; i<p_packet->rx.param_length; i++)
        {
          p_resp->p_node[p_resp->id_count]->p_data[i] = p_packet->rx.p_param[i];
        }

        p_resp->id_count++;
        dxl_ret = DXL_RET_RX_RESP;
        break;
      }
      else if (dxl_ret != DXL_RET_EMPTY)
      {
        break;
      }
      else
      {
#ifdef _USE_HW_RTOS
        osThreadYield();
#endif
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdWrite(dxl_t *p_packet, uint8_t id, uint8_t *p_data, uint16_t addr, uint16_t length, dxlcmd_resp_write_t *p_resp, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;
  uint8_t  tx_param[2 + DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE];
  uint16_t tx_length;
  uint32_t i;

  /*
  if (id == DXL_GLOBAL_ID)
  {
    return DXL_RET_ERROR_NOT_GLOBALID;
  }
  */

  if (dxlIsOpen(p_packet) == true)
  {
    if (length > DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE)
    {
      return DXL_RET_ERROR_LENGTH;
    }

    if (p_packet->packet_ver == DXL_PACKET_VER_1_0 )
    {
      tx_param[0] = addr;

      for (i=0; i<length; i++)
      {
        tx_param[1 + i] = p_data[i];
      }

      tx_length = 1 + length;
    }
    else
    {
      tx_param[0] = addr >> 0;
      tx_param[1] = addr >> 8;

      for (i=0; i<length; i++)
      {
        tx_param[2 + i] = p_data[i];
      }

      tx_length = 2 + length;
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_WRITE, tx_param, tx_length);
    p_packet->tx_time = micros() - pre_time_us;


    if (id == DXL_GLOBAL_ID)
    {
      return dxl_ret;
    }

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;

        p_resp->id    = p_packet->rx.id;
        p_resp->error = p_packet->rx.error;

        dxl_ret = DXL_RET_RX_RESP;
        break;
      }
      else if (dxl_ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdWriteNoResp(dxl_t *p_packet, uint8_t id, uint8_t *p_data, uint16_t addr, uint16_t length)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint8_t  tx_param[2 + DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE];
  uint16_t tx_length;
  uint32_t i;


  if (dxlIsOpen(p_packet) == true)
  {
    if (length > DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE)
    {
      return DXL_RET_ERROR_LENGTH;
    }

    if (p_packet->packet_ver == DXL_PACKET_VER_1_0 )
    {
      tx_param[0] = addr;

      for (i=0; i<length; i++)
      {
        tx_param[1 + i] = p_data[i];
      }

      tx_length = 1 + length;
    }
    else
    {
      tx_param[0] = addr >> 0;
      tx_param[1] = addr >> 8;

      for (i=0; i<length; i++)
      {
        tx_param[2 + i] = p_data[i];
      }

      tx_length = 2 + length;
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_WRITE, tx_param, tx_length);
    p_packet->tx_time = micros() - pre_time_us;
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdSyncRead(dxl_t *p_packet, dxlcmd_param_sync_read_t *p_param, dxlcmd_resp_read_t *p_resp, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[4 + DXLCMD_MAX_NODE];
  uint32_t mem_addr;
  uint8_t  *p_mem = (uint8_t *)p_resp->mem;
  uint32_t i;


  p_resp->id_count = 0;
  p_resp->p_node[0] = (dxlcmd_read_node_t *)&p_mem[0];


  if (dxlIsOpen(p_packet) == true)
  {
    tx_param[0] = p_param->addr >> 0;
    tx_param[1] = p_param->addr >> 8;
    tx_param[2] = p_param->length >> 0;
    tx_param[3] = p_param->length >> 8;

    for( i=0; i<p_param->id_count; i++)
    {
      tx_param[4+i] = p_param->id_tbl[i];
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, DXL_GLOBAL_ID, INST_SYNC_READ, tx_param, 4 + p_param->id_count);
    p_packet->tx_time = micros() - pre_time_us;

    mem_addr = 0;
    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;

        //-- 주소를 4바이트로 정렬( 구조체를 직접 타입변환하여 사용하기 위해서 )
        //
        mem_addr += sizeof(dxlcmd_read_node_t);
        if (mem_addr%4)
        {
          mem_addr += 4 - (mem_addr%4);
        }
        p_resp->p_node[p_resp->id_count]->p_data = &p_mem[mem_addr];

        p_resp->p_node[p_resp->id_count]->id     = p_packet->rx.id;
        p_resp->p_node[p_resp->id_count]->error  = p_packet->rx.error;
        p_resp->p_node[p_resp->id_count]->length = p_packet->rx.param_length;

        for (i=0; i<p_packet->rx.param_length; i++)
        {
          p_resp->p_node[p_resp->id_count]->p_data[i] = p_packet->rx.p_param[i];
        }

        p_resp->id_count++;

        mem_addr += p_packet->rx.param_length;
        p_resp->p_node[p_resp->id_count] = (dxlcmd_read_node_t *)&p_mem[mem_addr];

        if (p_resp->id_count >= p_param->id_count)
        {
          dxl_ret = DXL_RET_RX_RESP;
          break;
        }
      }
      else
      {
#ifdef _USE_HW_RTOS
        osThreadYield();
#endif
      }

      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdBulkRead(dxl_t *p_packet, dxlcmd_param_bulk_read_t *p_param, dxlcmd_resp_read_t *p_resp, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[DXLCMD_MAX_NODE * 5];
  uint32_t mem_addr;
  uint8_t *p_mem = (uint8_t *)p_resp->mem;
  uint32_t i;
  uint16_t tx_length;
#ifdef _USE_HW_RTOS
  //uint32_t pre_time;

  //pre_time = osKernelSysTick();
#endif

  p_resp->id_count = 0;
  p_resp->p_node[0] = (dxlcmd_read_node_t *)&p_mem[0];


  if (dxlIsOpen(p_packet) == true)
  {
    tx_length = 0;
    for( i=0; i<p_param->id_count; i++)
    {
      tx_param[tx_length+0] = p_param->id_tbl[i];
      tx_param[tx_length+1] = p_param->addr[i] >> 0;
      tx_param[tx_length+2] = p_param->addr[i] >> 8;
      tx_param[tx_length+3] = p_param->length[i] >> 0;
      tx_param[tx_length+4] = p_param->length[i] >> 8;
      tx_length += 5;
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, DXL_GLOBAL_ID, INST_BULK_READ, tx_param, tx_length);
    p_packet->tx_time = micros() - pre_time_us;

    mem_addr = 0;
    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;

        mem_addr += sizeof(dxlcmd_read_node_t);
        p_resp->p_node[p_resp->id_count]->p_data = &p_mem[mem_addr];

        p_resp->p_node[p_resp->id_count]->id     = p_packet->rx.id;
        p_resp->p_node[p_resp->id_count]->error  = p_packet->rx.error;
        p_resp->p_node[p_resp->id_count]->length = p_packet->rx.param_length;

        for (i=0; i<p_packet->rx.param_length; i++)
        {
          p_resp->p_node[p_resp->id_count]->p_data[i] = p_packet->rx.p_param[i];
        }

        p_resp->id_count++;

        mem_addr += p_packet->rx.param_length;

        //-- 주소를 4바이트로 정렬( 구조체를 직접 타입변환하여 사용하기 위해서 )
        //
        mem_addr += sizeof(dxlcmd_read_node_t);
        if (mem_addr%4)
        {
          mem_addr += 4 - (mem_addr%4);
        }

        p_resp->p_node[p_resp->id_count] = (dxlcmd_read_node_t *)&p_mem[mem_addr];

        if (p_resp->id_count >= p_param->id_count)
        {
          dxl_ret = DXL_RET_RX_RESP;
          break;
        }
      }
      else
      {
#ifdef _USE_HW_RTOS
        //osDelayUntil(&pre_time, 1);
        osThreadYield();
#endif
      }

      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdSyncWrite(dxl_t *p_packet, dxlcmd_param_sync_write_t *p_param)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;

  uint8_t tx_param[4 + DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE];
  uint32_t i;
  uint32_t j;
  uint32_t data_index;


  if (dxlIsOpen(p_packet) == true)
  {

    if (p_param->length > DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE)
    {
      return DXL_RET_ERROR_LENGTH;
    }

    if (p_packet->packet_ver == DXL_PACKET_VER_1_0 )
    {
      tx_param[0] = p_param->addr;
      tx_param[1] = p_param->length;

      data_index = 2;
      for( i=0; i<p_param->id_count; i++)
      {
        tx_param[data_index++] = p_param->node[i].id;
        for (j=0; j<p_param->length; j++)
        {
          tx_param[data_index++] = p_param->node[i].data[j];
        }
      }
    }
    else
    {
      tx_param[0] = p_param->addr >> 0;
      tx_param[1] = p_param->addr >> 8;
      tx_param[2] = p_param->length >> 0;
      tx_param[3] = p_param->length >> 8;

      data_index = 4;
      for( i=0; i<p_param->id_count; i++)
      {
        tx_param[data_index++] = p_param->node[i].id;
        for (j=0; j<p_param->length; j++)
        {
          tx_param[data_index++] = p_param->node[i].data[j];
        }
      }
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, DXL_GLOBAL_ID, INST_SYNC_WRITE, tx_param, data_index);
    p_packet->tx_time = micros() - pre_time_us;
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdBulkWrite(dxl_t *p_packet, dxlcmd_param_bulk_write_t *p_param)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;

  uint8_t tx_param[4 + DXLCMD_MAX_NODE];
  uint32_t i;
  uint32_t j;
  uint32_t data_index;
  uint32_t tx_buf_length;


  if (dxlIsOpen(p_packet) == true)
  {
    tx_buf_length = sizeof(tx_param);
    data_index = 0;

    for( i=0; i<p_param->id_count; i++)
    {
      tx_param[data_index++] = p_param->node[i].id;
      tx_param[data_index++] = p_param->node[i].addr >> 0;
      tx_param[data_index++] = p_param->node[i].addr >> 8;
      tx_param[data_index++] = p_param->node[i].length >> 0;
      tx_param[data_index++] = p_param->node[i].length >> 8;
      for (j=0; j<p_param->node[i].length; j++)
      {
        tx_param[data_index++] = p_param->node[i].data[j];
      }

      if (data_index > tx_buf_length)
      {
        return DXL_RET_ERROR_LENGTH;
      }
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, DXL_GLOBAL_ID, INST_BULK_WRITE, tx_param, data_index);
    p_packet->tx_time = micros() - pre_time_us;
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdMemoryErase(dxl_t *p_packet, uint8_t id, uint32_t addr, uint32_t length, err_code_t *p_err, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[8];


  *p_err = ERR_NONE;


  if (id == DXL_GLOBAL_ID)
  {
    return DXL_RET_ERROR_NOT_GLOBALID;
  }

  if (dxlIsOpen(p_packet) == true)
  {
    tx_param[0] = addr >> 0;
    tx_param[1] = addr >> 8;
    tx_param[2] = addr >> 16;
    tx_param[3] = addr >> 24;

    tx_param[4] = length >> 0;
    tx_param[5] = length >> 8;
    tx_param[6] = length >> 16;
    tx_param[7] = length >> 24;

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_MEMORY_ERASE, tx_param, 8);
    p_packet->tx_time = micros() - pre_time_us;

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;


        if (p_packet->rx.error == DXL_RET_OK)
        {
          *p_err  = p_packet->rx.p_param[0]<<0;
          *p_err |= p_packet->rx.p_param[1]<<8;
          *p_err |= p_packet->rx.p_param[2]<<16;
          *p_err |= p_packet->rx.p_param[3]<<24;

          dxl_ret = DXL_RET_RX_RESP;
        }
        else
        {
          dxl_ret = DXL_RET_ERROR;
        }
        break;
      }
      else if (dxl_ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  if (dxl_ret != DXL_RET_RX_RESP)
  {
    *p_err = ERR_DXL_ERROR + dxl_ret;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdMemoryRead(dxl_t *p_packet, uint8_t id, uint32_t addr, uint8_t *p_data, uint32_t length, err_code_t *p_err, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[8];
  uint32_t i;


  *p_err = ERR_NONE;


  if (id == DXL_GLOBAL_ID)
  {
    return DXL_RET_ERROR_NOT_GLOBALID;
  }

  if (dxlIsOpen(p_packet) == true)
  {
    tx_param[0] = addr >> 0;
    tx_param[1] = addr >> 8;
    tx_param[2] = addr >> 16;
    tx_param[3] = addr >> 24;

    tx_param[4] = length >> 0;
    tx_param[5] = length >> 8;
    tx_param[6] = length >> 16;
    tx_param[7] = length >> 24;

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_MEMORY_READ, tx_param, 8);
    p_packet->tx_time = micros() - pre_time_us;

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;


        if (p_packet->rx.error == DXL_RET_OK)
        {
          *p_err  = p_packet->rx.p_param[0]<<0;
          *p_err |= p_packet->rx.p_param[1]<<8;
          *p_err |= p_packet->rx.p_param[2]<<16;
          *p_err |= p_packet->rx.p_param[3]<<24;

          for (i=4; i<p_packet->rx.param_length; i++)
          {
            p_data[i-4] = p_packet->rx.p_param[i];
          }
          dxl_ret = DXL_RET_RX_RESP;
        }
        else
        {
          dxl_ret = DXL_RET_ERROR;
        }
        break;
      }
      else if (dxl_ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  if (dxl_ret != DXL_RET_RX_RESP)
  {
    *p_err = ERR_DXL_ERROR + dxl_ret;
  }

  return dxl_ret;
}

dxl_error_t dxlcmdMemoryWrite(dxl_t *p_packet, uint8_t id, uint32_t addr, uint8_t *p_data, uint32_t length, err_code_t *p_err, uint32_t timeout)
{
  dxl_error_t dxl_ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[DXLCMD_MAX_BUFFER+8];
  uint32_t i;


  *p_err = ERR_NONE;


  if (id == DXL_GLOBAL_ID)
  {
    return DXL_RET_ERROR_NOT_GLOBALID;
  }

  if (dxlIsOpen(p_packet) == true)
  {
    tx_param[0] = addr >> 0;
    tx_param[1] = addr >> 8;
    tx_param[2] = addr >> 16;
    tx_param[3] = addr >> 24;

    tx_param[4] = length >> 0;
    tx_param[5] = length >> 8;
    tx_param[6] = length >> 16;
    tx_param[7] = length >> 24;


    if (length > DXLCMD_MAX_BUFFER+8)
    {
      return DXL_RET_ERROR_LENGTH;
    }

    for (i=0; i<length; i++)
    {
      tx_param[8+i] = p_data[i];
    }

    pre_time_us = micros();
    dxl_ret = dxlTxPacketInst(p_packet, id, INST_MEMORY_WRITE, tx_param, 8+length);
    p_packet->tx_time = micros() - pre_time_us;

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      dxl_ret = dxlRxPacket(p_packet);
      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        p_packet->rx_time = micros() - pre_time_us;


        if (p_packet->rx.error == DXL_RET_OK)
        {
          *p_err  = p_packet->rx.p_param[0]<<0;
          *p_err |= p_packet->rx.p_param[1]<<8;
          *p_err |= p_packet->rx.p_param[2]<<16;
          *p_err |= p_packet->rx.p_param[3]<<24;

          dxl_ret = DXL_RET_RX_RESP;
        }
        else
        {
          dxl_ret = DXL_RET_ERROR;
        }
        break;
      }
      else if (dxl_ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {

        break;
      }
    }
  }
  else
  {
    dxl_ret = DXL_RET_NOT_OPEN;
  }

  if (dxl_ret != DXL_RET_RX_RESP)
  {
    *p_err = ERR_DXL_ERROR + dxl_ret;
  }

  return dxl_ret;
}


data_t getDataType(uint8_t *p_data, uint8_t length)
{
  data_t ret;
  uint32_t i;


  ret.u32Data = 0;

  for (i=0; i<length; i++)
  {
    ret.u8Data[i] = p_data[i];
  }

  return ret;
}

