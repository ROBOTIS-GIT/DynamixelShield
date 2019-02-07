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
 * Version: 0.0.4
 * Authors: Hancheol Cho (Baram) 
 *          KyungWan Ki  (Kei)
 */

#ifndef DYNAMIXEL_SHIELD_H_
#define DYNAMIXEL_SHIELD_H_

#include <stdint.h>
#include "./util/dxl.h"
#include "./util/dxl_cmd.h"
#include "./util/RC100.h"
#include "Stream.h"




#define M_AX      ((uint8_t)0)
#define M_RX      ((uint8_t)1)
#define M_EX      ((uint8_t)2)
#define M_MX      ((uint8_t)3)
#define M_MX2     ((uint8_t)4)
#define M_XL320   ((uint8_t)5)
#define M_XL430   ((uint8_t)6)
#define M_XM430   ((uint8_t)7)
#define M_XM540   ((uint8_t)8)

#define M_AX12    M_AX
#define M_AX18    M_AX
#define M_RX24    M_RX
#define M_RX28    M_RX
#define M_RX64    M_RX
#define M_EX106   M_EX
#define M_MX28    M_MX
#define M_MX64    M_MX
#define M_MX106   M_MX
#define M_MX28_2  M_MX2
#define M_MX64_2  M_MX2
#define M_MX106_2 M_MX2

#define M_BAUD_TYPE_0         0   // AX ~
#define M_BAUD_TYPE_1         1   // XL320
#define M_BAUD_TYPE_2         2   // X ~

typedef struct
{
  uint16_t addr;
  uint8_t  length;
} dxl_model_addr_t;

typedef struct
{
  uint8_t  protocol;
  uint8_t  id_index;
  uint8_t  baud_type;

  dxl_model_addr_t goal_pos;  
  dxl_model_addr_t cur_pos;  
  dxl_model_addr_t goal_speed;  
  dxl_model_addr_t cur_speed;    
  dxl_model_addr_t torque;  
  dxl_model_addr_t led;  
  dxl_model_addr_t op_mode;  
  uint8_t          op_joint_mode;
  uint8_t          op_wheel_mode;
  uint16_t         max_res;
  uint16_t         max_angle;

  dxl_model_addr_t id;  
  dxl_model_addr_t baud;  
} dxl_model_t;



typedef struct
{
  uint8_t  id_count;
  uint8_t  id_info[DXLCMD_MAX_NODE];
  uint16_t id_model[DXLCMD_MAX_NODE];
} dxl_info_t;


class DynamixelShield : public Stream
{
private:    
  dxl_t           dxl_cmd;
  dxlcmd_param_t  param;
  dxlcmd_resp_t   resp;
  dxl_info_t      dxl_info;

  uint32_t        err_code;

  bool            sync_write_enable;
  uint16_t        sync_write_version;

public:
  RC100 rc100;

public:
  DynamixelShield();
  ~DynamixelShield();

  bool begin(uint32_t baud_rate = 57600, uint8_t protocol_version = DXL_PACKET_VER_2_0);
   
  bool scan(void);
  bool ping(uint8_t id = DXL_GLOBAL_ID);
  bool addMotor(uint8_t id, uint8_t model);
  bool setProtocolVersion(uint8_t version);
  bool write(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t length, uint32_t timeout);
  bool read(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t length, uint32_t timeout);

  uint8_t getDxlCount(void);
  uint8_t getDxlID(uint8_t index);
  uint32_t getErr(void);
  void clearErr(void);

  bool reboot(uint8_t id);
  bool reset(uint8_t id);

  bool setID(uint8_t id, uint8_t new_id);
  bool setBaud(uint8_t id, uint32_t new_baud);

  bool ledOn(uint8_t id);
  bool ledOff(uint8_t id);

  bool torqueOn(uint8_t id);
  bool torqueOff(uint8_t id);

  bool setJointMode(uint8_t id);
  bool setWheelMode(uint8_t id);
  
  bool    setGoalPosition(uint8_t id, uint32_t position);
  int32_t getGoalPosition(uint8_t id);
  int32_t getCurPosition(uint8_t id);

  bool    setGoalSpeed(uint8_t id, int32_t speed);
  int32_t getGoalSpeed(uint8_t id);   
  int32_t getCurSpeed(uint8_t id);  

  bool    setGoalAngle(uint8_t id, int32_t angle);
  int32_t getGoalAngle(uint8_t id); 
  int32_t getCurAngle(uint8_t id);

  bool syncWriteBegin(void);
  bool syncWriteEnd(void);

  int available(void);
  int peek(void);
  int read(void);
  void flush(void);
  size_t write(const uint8_t c);
  using Print::write; // pull in write(str) and write(buf, size) from Print

private:
  void getDxlModel(uint8_t model_index, dxl_model_t *p_model);
  uint8_t getDxlModelIndex(uint16_t model_number);
  uint8_t getDxlIdIndex(uint8_t id);
  bool getDxlModelFromID(uint8_t id, dxl_model_t *p_model);
  int16_t getBaudIndex(uint32_t baud, uint8_t baud_type);

  int32_t getPosFromAngle(dxl_model_t *p_model, int32_t angle);
  int32_t getAngleFromPos(dxl_model_t *p_model, int32_t pos);

  bool addSyncWrite(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t length);  

};

#endif 
