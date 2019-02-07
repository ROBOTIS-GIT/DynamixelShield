/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* 
 * Authors: Hancheol Cho (Baram) 
 *          KyungWan Ki  (Kei)
 */

#include "DynamixelShield.h"


#ifndef SoftwareSerial_h
#pragma message("\r\nWarning : You can't use the RC100 function, because this board doesn't have SoftwareSerial.h")
#endif





DynamixelShield::DynamixelShield()
{
  dxl_info.id_count = 0;
  err_code = ERR_NONE;
  sync_write_enable = false;
  param.sync_write.id_count = 0;
  param.sync_write.addr = 0;
  param.sync_write.length = 0;      
}

DynamixelShield::~DynamixelShield()
{

}

bool DynamixelShield::begin(uint32_t baud_rate, uint8_t protocol_version)
{
  bool ret = true;
  
  dxlInit(&dxl_cmd, protocol_version);
  dxlSetSerial(&dxl_cmd, &DXL_PORT);
  dxlSetDirPin(&dxl_cmd, 2);
  dxlOpenPort(&dxl_cmd, baud_rate);      

#ifdef SoftwareSerial_h
  rc100.begin();
#endif

  dxl_info.id_count = 0;
  sync_write_enable = false;
  param.sync_write.id_count = 0;
  param.sync_write.addr = 0;
  param.sync_write.length = 0;      
  return ret;
}

bool DynamixelShield::scan(void)
{
  bool ret = true;

  ret = ping(DXL_GLOBAL_ID);

  return ret;
}

bool DynamixelShield::addMotor(uint8_t id, uint8_t model)
{
  bool ret = true;

  if (dxl_info.id_count < DXLCMD_MAX_NODE)
  {
    dxl_info.id_info[dxl_info.id_count] = id;
    dxl_info.id_model[dxl_info.id_count] = model;

    dxl_info.id_count++;
  }
  else
  {
    ret = false;
  }

  return ret;
}

bool DynamixelShield::setProtocolVersion(uint8_t version)
{
  if (version == DXL_PACKET_VER_1_0)
  {
    dxlSetProtocolVersion(&dxl_cmd, DXL_PACKET_VER_1_0);
  }
  else
  {
    dxlSetProtocolVersion(&dxl_cmd, DXL_PACKET_VER_2_0);
  }
  return true;
}

bool DynamixelShield::ping(uint8_t id)
{
  bool ret = true;
  dxl_error_t dxl_err;    
  uint8_t i;
  uint8_t id_i;
  uint8_t dxl_cnt;


  if (id == DXL_GLOBAL_ID)
  {
    dxl_info.id_count = 0;

    dxl_err = dxlcmdPing(&dxl_cmd, DXL_GLOBAL_ID, &resp.ping, 500);  
    dxl_cnt = resp.ping.id_count;

    for (i=0; i<dxl_cnt; i++)
    {
      id_i = resp.ping.p_node[i]->id;
      dxl_err = dxlcmdRead(&dxl_cmd, id_i, 0, 2, &resp.read, 20);

      if (dxl_err == DXL_RET_RX_RESP)
      {        
        if (dxl_info.id_count < DXLCMD_MAX_NODE)
        {
          dxl_info.id_info[dxl_info.id_count]  = id_i;
          dxl_info.id_model[dxl_info.id_count] = getDxlModelIndex(resp.read.p_node[0]->p_data[1]<<8 | resp.read.p_node[0]->p_data[0]<<0);
          dxl_info.id_count++;      
        }
        err_code = ERR_NONE;
      }
    }

    if (dxl_info.id_count == 0)
    {
      ret = false;
    }
  }
  else
  {
    dxl_err = dxlcmdPing(&dxl_cmd, id, &resp.ping, 20);  

    if (dxl_err == DXL_RET_RX_RESP && resp.ping.id_count > 0)
    {        
      if (dxl_info.id_count < DXLCMD_MAX_NODE)
      {
        for (int i=0; i<dxl_info.id_count; i++)
        {
          if (dxl_info.id_info[i] == id)
          {
            break;
          }  
        }
        if (i == dxl_info.id_count)
        {
          dxl_info.id_info[dxl_info.id_count] = id;
          dxl_info.id_count++;      
        }
      }
      err_code = ERR_NONE;     
    }
    if (dxl_info.id_count == 0)
    {
      ret = false;
    }    
  }

  return ret;
}

uint8_t DynamixelShield::getDxlCount(void)
{
  return dxl_info.id_count;
}

uint8_t DynamixelShield::getDxlID(uint8_t index)
{
  if (index >= DXLCMD_MAX_NODE)
  {
    return 0;
  }

  return dxl_info.id_info[index];
}

uint8_t DynamixelShield::getDxlIdIndex(uint8_t id)
{
  uint8_t index = 0;

  for (int i=0; i<dxl_info.id_count; i++)
  {
    if (dxl_info.id_info[i] == id)
    {
      index = i;
    }
  }

  return index;
}

uint32_t DynamixelShield::getErr(void)
{
  return err_code;
}
  
void DynamixelShield::clearErr(void)
{
  err_code = ERR_NONE;
}

bool DynamixelShield::reboot(uint8_t id)
{
  bool ret = true;


  return ret;
}

bool DynamixelShield::reset(uint8_t id)
{
  bool ret = true;


  return ret;
}

bool DynamixelShield::setID(uint8_t id, uint8_t new_id)
{
  bool ret;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    ret = write(id, dxl_model.id.addr, (uint8_t *)&new_id, dxl_model.id.length, 100);
  }

  return ret;
}

bool DynamixelShield::setBaud(uint8_t id, uint32_t new_baud)
{
  bool ret;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    int16_t baud_index;

    baud_index = getBaudIndex(new_baud, dxl_model.baud_type);
    if (baud_index >= 0)
    {
      new_baud = baud_index;
      ret = write(id, dxl_model.baud.addr, (uint8_t *)&new_baud, dxl_model.baud.length, 100);
    }
  }

  return ret;
}

int16_t DynamixelShield::getBaudIndex(uint32_t baud, uint8_t baud_type)
{
  int16_t baud_index = -1;
  
  switch(baud)
  {
    case 4000000:
      if (baud_type == M_BAUD_TYPE_2)
      {
        baud_index = 6;
      }
      break;

    case 3000000:
      if (baud_type == M_BAUD_TYPE_2)
      {
        baud_index = 5;
      }
      break;

    case 2000000:
      if (baud_type == M_BAUD_TYPE_2)
      {
        baud_index = 4;
      }
      break;

    case 1000000:
      if (baud_type == M_BAUD_TYPE_0)
      {
        baud_index = 1;
      }
      if (baud_type == M_BAUD_TYPE_1)
      {
        baud_index = 3;
      }
      if (baud_type == M_BAUD_TYPE_2)
      {
        baud_index = 3;
      }
      break;

    case 115200:
      if (baud_type == M_BAUD_TYPE_0)
      {
        baud_index = 16;
      }
      if (baud_type == M_BAUD_TYPE_1)
      {
        baud_index = 2;
      }
      if (baud_type == M_BAUD_TYPE_2)
      {
        baud_index = 2;
      }
      break;

    case 57600:
      if (baud_type == M_BAUD_TYPE_0)
      {
        baud_index = 34;
      }
      if (baud_type == M_BAUD_TYPE_1)
      {
        baud_index = 1;
      }      
      if (baud_type == M_BAUD_TYPE_2)
      {
        baud_index = 1;
      }
      break;

    case 9600:
      if (baud_type == M_BAUD_TYPE_0)
      {
        baud_index = 207;
      }
      if (baud_type == M_BAUD_TYPE_1)
      {
        baud_index = 0;
      }
      if (baud_type == M_BAUD_TYPE_2)
      {
        baud_index = 0;
      }
      break;
  }

  return baud_index;
}

bool DynamixelShield::write(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t length, uint32_t timeout)
{
  bool ret = true;
  uint32_t data;
  dxl_error_t dxl_ret;


  dxl_ret = dxlcmdWrite(&dxl_cmd, id, p_data, addr, length, &resp.write, timeout); 
  if (dxl_ret != DXL_RET_RX_RESP && dxl_ret != DXL_RET_OK)
  {
    err_code = dxl_ret;
    ret = false;
  }

  return ret;
}

bool DynamixelShield::read(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t length, uint32_t timeout)
{
  bool ret = true;
  dxl_error_t dxl_ret;


  if (ret == true)
  {
    dxl_ret = dxlcmdRead(&dxl_cmd, id, addr, length, &resp.read, 100); 
    if (dxl_ret == DXL_RET_RX_RESP && resp.read.id_count > 0)
    {
      for (int i=0; i<length; i++)
      {
        p_data[i] = resp.read.p_node[0]->p_data[i];
      }    
    }
    else
    {
      err_code = dxl_ret;
      ret = false;
    }
  }

  return ret;
}


bool DynamixelShield::ledOn(uint8_t id)
{
  bool ret;
  uint8_t data;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 1;
    ret = write(id, dxl_model.led.addr, (uint8_t *)&data, dxl_model.led.length, 100);
  }

  return ret;
}

bool DynamixelShield::ledOff(uint8_t id)
{
  bool ret;
  uint8_t data;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 0;
    ret = write(id, dxl_model.led.addr, (uint8_t *)&data, dxl_model.led.length, 100);
  }

  return ret;
}

bool DynamixelShield::torqueOn(uint8_t id)
{
  bool ret;
  uint8_t data;  
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 1;
    ret = write(id, dxl_model.torque.addr, (uint8_t *)&data, dxl_model.torque.length, 100);
  }

  return ret;
}

bool DynamixelShield::torqueOff(uint8_t id)
{
  bool ret;
  uint8_t data;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 0;
    ret = write(id, dxl_model.torque.addr, (uint8_t *)&data, dxl_model.torque.length, 100);
  }

  return ret;
}

bool DynamixelShield::setJointMode(uint8_t id)
{
  bool ret;
  uint8_t data;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    if (dxl_model.protocol == DXL_PACKET_VER_2_0)
    {
      data = dxl_model.op_joint_mode;      
      ret = write(id, dxl_model.op_mode.addr, (uint8_t *)&data, dxl_model.op_mode.length, 100);
    }
  }

  return ret;
}

bool DynamixelShield::setWheelMode(uint8_t id)
{
  bool ret;
  uint8_t data;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    if (dxl_model.protocol == DXL_PACKET_VER_2_0)
    {
      data = dxl_model.op_wheel_mode;
      ret = write(id, dxl_model.op_mode.addr, (uint8_t *)&data, dxl_model.op_mode.length, 100);
    }
  }

  return ret;
}

bool DynamixelShield::addSyncWrite(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t length)
{
  if (param.sync_write.id_count >= DXLCMD_MAX_NODE)
  {
    return false;
  }

  if (param.sync_write.addr == 0 || param.sync_write.addr == addr)
  {
    param.sync_write.addr = addr;
    param.sync_write.length = length;
    param.sync_write.node[param.sync_write.id_count].id = id;
    for (uint8_t j=0; j<length; j++)
    {
      param.sync_write.node[param.sync_write.id_count].data[j] = p_data[j];
    }
    param.sync_write.id_count++;        
  }

  return true;
}

bool DynamixelShield::setGoalPosition(uint8_t id, uint32_t position)
{
  bool ret;
  uint32_t data;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = position;

    if (sync_write_enable == true)
    {      
      sync_write_version = dxl_model.protocol;      
      addSyncWrite(id, dxl_model.goal_pos.addr, (uint8_t *)&data, dxl_model.goal_pos.length);
    }
    else
    {
      ret = write(id, dxl_model.goal_pos.addr, (uint8_t *)&data, dxl_model.goal_pos.length, 100);
    }
  }

  return ret;
}

bool DynamixelShield::setGoalSpeed(uint8_t id, int32_t speed)
{
  bool ret;
  int32_t data;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = speed;

    if (sync_write_enable == true)
    {      
      sync_write_version = dxl_model.protocol;      
      addSyncWrite(id, dxl_model.goal_pos.addr, (uint8_t *)&data, dxl_model.goal_pos.length);
    }
    else
    {    
      ret = write(id, dxl_model.goal_speed.addr, (uint8_t *)&data, dxl_model.goal_speed.length, 100);
    }
  }

  return true;
}

int32_t DynamixelShield::getGoalPosition(uint8_t id)
{
  bool ret;
  int32_t data = 0;
  int32_t ret_data = 0;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 0;
    ret = read(id, dxl_model.goal_pos.addr, (uint8_t *)&data, dxl_model.goal_pos.length, 100);

    if (ret == true)
    {
      if (dxl_model.goal_pos.length == 2)
      {
        ret_data = (int16_t)data;
      }
      else
      {
        ret_data = data;
      }      
    }
  }

  return ret_data;
}

int32_t DynamixelShield::getGoalSpeed(uint8_t id)
{
  bool ret;
  int32_t data = 0;
  int32_t ret_data = 0;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 0;
    ret = read(id, dxl_model.goal_speed.addr, (uint8_t *)&data, dxl_model.goal_speed.length, 100);
    if (ret == true)
    {
      if (dxl_model.goal_speed.length == 2)
      {
        ret_data = (int16_t)data;
      }
      else
      {
        ret_data = data;
      }
    }
  }

  return ret_data;
}

int32_t DynamixelShield::getCurPosition(uint8_t id)
{
  bool ret;
  int32_t data = 0;
  int32_t ret_data = 0;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 0;    
    ret = read(id, dxl_model.cur_pos.addr, (uint8_t *)&data, dxl_model.cur_pos.length, 100);
    if (ret == true)
    {
      if (dxl_model.cur_pos.length == 2)
      {
        ret_data = (int16_t)data;
      }
      else
      {
        ret_data = data;
      }      
    }
  }

  return ret_data;
}

int32_t DynamixelShield::getCurSpeed(uint8_t id)
{
  bool ret;
  int32_t data = 0;
  int32_t ret_data = 0;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    data = 0;
    ret = read(id, dxl_model.cur_speed.addr, (uint8_t *)&data, dxl_model.cur_speed.length, 100);
    if (ret == true)
    {
      if (dxl_model.cur_speed.length == 2)
      {
        ret_data = (int16_t)data;
      }
      else
      {
        ret_data = data;
      }
    }
  }

  return ret_data;
}

int32_t DynamixelShield::getPosFromAngle(dxl_model_t *p_model, int32_t angle)
{
  int32_t pos_angle;

  angle += p_model->max_angle / 2;
  pos_angle = angle * p_model->max_res / p_model->max_angle;
  pos_angle = constrain(pos_angle, 0, (p_model->max_res - 1));

  
  return pos_angle;
}

int32_t DynamixelShield::getAngleFromPos(dxl_model_t *p_model, int32_t pos)
{
  int32_t angle;

  pos = pos - (p_model->max_res / 2);
  angle = pos * p_model->max_angle / p_model->max_res;
    
  return angle;
}

bool DynamixelShield::setGoalAngle(uint8_t id, int32_t angle)
{
  bool ret;
  int32_t pos;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == true)
  {
    pos = getPosFromAngle(&dxl_model, angle);
    ret = write(id, dxl_model.goal_pos.addr, (uint8_t *)&pos, dxl_model.goal_pos.length, 100);    
  }
  
  return ret;
}

int32_t DynamixelShield::getGoalAngle(uint8_t id)
{
  bool ret;
  int32_t pos;
  int32_t angle = 0;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == ret)
  {
    pos   = getGoalPosition(id);
    angle = getAngleFromPos(&dxl_model, pos);
  }

  return angle;
} 

int32_t DynamixelShield::getCurAngle(uint8_t id)
{
  bool ret;
  int32_t pos;
  int32_t angle = 0;
  dxl_model_t dxl_model;

  ret = getDxlModelFromID(id, &dxl_model);

  if (ret == ret)
  {
    pos   = getCurPosition(id);
    angle = getAngleFromPos(&dxl_model, pos);
  }

  return angle;
}

bool DynamixelShield::syncWriteBegin(void)
{
  bool ret = true;

  if (dxl_info.id_count > 0)
  {
    sync_write_enable  = true;

    param.sync_write.id_count = 0;
    param.sync_write.addr     = 0;
    param.sync_write.length   = 0;    
  }
  return ret;
}

bool DynamixelShield::syncWriteEnd(void)
{
  bool ret = true;
  dxl_error_t dxl_ret;    


  if (sync_write_enable == true && param.sync_write.id_count > 0)
  {
    dxlSetProtocolVersion(&dxl_cmd, sync_write_version);
    dxl_ret = dxlcmdSyncWrite(&dxl_cmd, &param.sync_write);
    
    if (dxl_ret != DXL_RET_OK)
    {
      err_code = dxl_ret;
      ret = false;
    }
  }

  sync_write_enable = false;

  param.sync_write.id_count = 0;
  param.sync_write.addr     = 0;
  param.sync_write.length   = 0;       

  return ret;
}


int DynamixelShield::available( void )
{
#ifdef SoftwareSerial_h
  return rc100.p_serial->available();
#endif
}

int DynamixelShield::peek( void )
{
#ifdef SoftwareSerial_h
  return rc100.p_serial->peek();
#endif
}

void DynamixelShield::flush( void )
{
#ifdef SoftwareSerial_h
  rc100.p_serial->flush();
#endif
}

int DynamixelShield::read( void )
{
#ifdef SoftwareSerial_h
  return rc100.p_serial->read();
#endif
}

size_t DynamixelShield::write( const uint8_t uc_data )
{
#ifdef SoftwareSerial_h
  return rc100.p_serial->write(uc_data);
#endif
}

uint8_t DynamixelShield::getDxlModelIndex(uint16_t model_number)
{
  uint8_t model_index = M_XL430;


  switch(model_number)
  {
    case 12:    // AX12+
    case 300:   // AX12W
    case 18:    // AX18
    case 24:    // RX-24F
    case 28:    // RX-28
    case 64:    // RX-64
      model_index = M_AX;
      break;

    case 106:   // EX-106
    case 107:   // EX-106+
      model_index = M_EX;
      break;

    case 29:    // MX-28
    case 310:   // MX-64
    case 320:   // MX-106
      model_index = M_MX;
      break;

    case 30:    // MX-28(2.0)
    case 311:   // MX-64(2.0)
    case 321:   // MX-106(2.0)
      model_index = M_MX2;
      break;

    case 350:   // XL320
      model_index = M_XL320;
      break;

    case 1060:  // XL430-W250
      model_index = M_XL430;
      break;

    case 1030:  // XM430-W210
    case 1020:  // XM430-W350
      model_index = M_XM430;
      break;

    case 1130:  // XM540-W150
    case 1120:  // XM540-W270
      model_index = M_XM540;
      break;      
  }

  return model_index;
}

bool DynamixelShield::getDxlModelFromID(uint8_t id, dxl_model_t *p_model)
{
  if (dxl_info.id_count == 0) return false;

  p_model->id_index = getDxlIdIndex(id);

  getDxlModel(dxl_info.id_model[p_model->id_index], p_model);
  
  dxlSetProtocolVersion(&dxl_cmd, p_model->protocol);

  return true;
}

void DynamixelShield::getDxlModel(uint8_t model_index, dxl_model_t *p_model)
{
  switch(model_index)
  {
    case M_AX:
    case M_EX:
    case M_MX:
      p_model->protocol = DXL_PACKET_VER_1_0;
      p_model->goal_pos.addr   = 30;
      p_model->goal_pos.length = 2;
      p_model->cur_pos.addr   = 36;
      p_model->cur_pos.length = 2;
      p_model->goal_speed.addr   = 32;
      p_model->goal_speed.length = 2;      
      p_model->cur_speed.addr   = 38;
      p_model->cur_speed.length = 2;      
      p_model->torque.addr   = 24;
      p_model->torque.length = 1;
      p_model->led.addr   = 25;
      p_model->led.length = 1;   
      p_model->op_mode.addr   = 25;
      p_model->op_mode.length = 1; 
      p_model->id.addr   = 3;
      p_model->id.length = 1;         
      p_model->baud.addr   = 4;
      p_model->baud.length = 1;       
      p_model->baud_type = M_BAUD_TYPE_0;        

      if (model_index == M_EX)
      {
        p_model->max_res = 4096;
        p_model->max_angle = 2509;        
      }
      else if (model_index == M_MX)
      {
        p_model->max_res = 4096;
        p_model->max_angle = 3600;          
      }
      else
      {
        p_model->max_res = 1024;
        p_model->max_angle = 3000;
      }
      break;

    case M_XL320:
      p_model->protocol = DXL_PACKET_VER_2_0;
      p_model->goal_pos.addr   = 30;
      p_model->goal_pos.length = 2;
      p_model->cur_pos.addr   = 37;
      p_model->cur_pos.length = 2;
      p_model->goal_speed.addr   = 32;
      p_model->goal_speed.length = 2;                  
      p_model->cur_speed.addr   = 39;
      p_model->cur_speed.length = 2;                  
      p_model->torque.addr   = 24;
      p_model->torque.length = 1;
      p_model->led.addr   = 25;
      p_model->led.length = 1;  
      p_model->op_mode.addr = 11;
      p_model->op_mode.length = 1;  
      p_model->op_joint_mode = 2;
      p_model->op_wheel_mode = 1;           
      p_model->max_res = 1024;
      p_model->max_angle = 3000;      
      p_model->id.addr   = 3;
      p_model->id.length = 1;         
      p_model->baud.addr   = 4;
      p_model->baud.length = 1;      
      p_model->baud_type = M_BAUD_TYPE_1;         
      break;

    default:
      p_model->protocol = DXL_PACKET_VER_2_0;
      p_model->goal_pos.addr   = 116;
      p_model->goal_pos.length = 4;      
      p_model->cur_pos.addr   = 132;
      p_model->cur_pos.length = 4;
      p_model->goal_speed.addr   = 104;
      p_model->goal_speed.length = 4;            
      p_model->cur_speed.addr   = 128;
      p_model->cur_speed.length = 4;                        
      p_model->torque.addr   = 64;
      p_model->torque.length = 1;
      p_model->led.addr   = 65;
      p_model->led.length = 1;       
      p_model->op_mode.addr = 11;
      p_model->op_mode.length = 1;  
      p_model->op_joint_mode = 3;
      p_model->op_wheel_mode = 1;   
      p_model->max_res = 4096;
      p_model->max_angle = 3600;          
      p_model->id.addr   = 7;
      p_model->id.length = 1;         
      p_model->baud.addr   = 8;
      p_model->baud.length = 1;    
      p_model->baud_type = M_BAUD_TYPE_2;                                 
      break;          
  }
}


