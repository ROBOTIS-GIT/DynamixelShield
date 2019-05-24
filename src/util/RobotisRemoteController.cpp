/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* 
 *  Authors: Taehoon Lim (Darby)
 *           Hancheol Cho (Baram) 
 *           Ashe Kim
 *           KyungWan Ki  (Kei)
 */

#include "RobotisRemoteController.h"


RobotisRemoteController::RobotisRemoteController(uint8_t rx_pin, uint8_t tx_pin)
  : p_sw_port(nullptr), p_hw_port(nullptr)
{
#ifdef SoftwareSerial_h
  p_sw_port = new SoftwareSerial(rx_pin, tx_pin);
#endif
  memset(&rc100_rx, 0, sizeof(rc100_rx));
}

// RobotisRemoteController::RobotisRemoteController(HardwareSerial& port)
//   : p_hw_port(&port), p_sw_port(nullptr)
// {
//   memset(&rc100_rx, 0, sizeof(rc100_rx));
// }

RobotisRemoteController::~RobotisRemoteController()
{
}

void RobotisRemoteController::begin()
{
#ifdef SoftwareSerial_h
  p_sw_port != nullptr ? p_sw_port->begin(57600) : (void)(p_sw_port);
#endif  
  rc100_rx.state = 0;
  rc100_rx.index = 0;
  rc100_rx.received = false;
  rc100_rx.released_event = false;
}

int RobotisRemoteController::available(void)
{
#ifdef SoftwareSerial_h  
  if (p_sw_port != nullptr){
    if(p_sw_port->available()){
      return rc100Update(p_sw_port->read());
    }
  }
#endif
  return 0;
}

uint16_t RobotisRemoteController::readData(void)
{
  return rc100_rx.data;
}

void RobotisRemoteController::writeRaw(uint8_t temp)
{
#ifdef SoftwareSerial_h
  p_sw_port != nullptr ? (void)p_sw_port->write(temp) : (void)(p_sw_port);
#endif
}

uint8_t RobotisRemoteController::readRaw(void)
{
  uint8_t ret = 0;
#ifdef SoftwareSerial_h  
  ret = p_sw_port != nullptr ? p_sw_port->read() : 0;
#endif  
  return ret;
}

bool RobotisRemoteController::availableEvent(void)
{
  bool ret = false;

  
  rc100_rx.released_event = false;

#ifdef SoftwareSerial_h
  if (p_sw_port != nullptr){
    if(p_sw_port->available()){
      rc100Update(p_sw_port->read());
      ret = rc100_rx.released_event;
    }
  }  
#endif
  return ret;
}

uint16_t RobotisRemoteController::readEvent(void)
{
  return rc100_rx.event_msg;
}

void RobotisRemoteController::clear(void)
{
#ifdef SoftwareSerial_h  
  while(p_sw_port->available())
  {
    p_sw_port->read();
  }  
#endif
}

void RobotisRemoteController::flush(void)
{
  clear();
}

bool RobotisRemoteController::rc100Update(uint8_t data)
{
  bool ret = false;
  static uint8_t save_data;
  static uint8_t inv_data;
  static uint32_t time_t;

  inv_data = ~data;

  if (millis()-time_t > 100)
  {
    rc100_rx.state = 0;
  }

  switch(rc100_rx.state)
  {
    case 0:
      if (data == 0xFF)
      {
        rc100_rx.state = 1;
        time_t = millis();
      }
      break;

    case 1:
      if (data == 0x55)
      {
        rc100_rx.state    = 2;
        rc100_rx.received = false;
        rc100_rx.data     = 0;
      }
      else
      {
        rc100_rx.state = 0;
      }
      break;

    case 2:
      rc100_rx.data  = data;
      save_data      = data;
      rc100_rx.state = 3;
      break;

    case 3:
      if (save_data == inv_data)
      {
        rc100_rx.state = 4;
      }
      else
      {
        rc100_rx.state = 0;
      }
      break;

    case 4:
      rc100_rx.data |= data<<8;
      save_data      = data;
      rc100_rx.state = 5;
      break;

    case 5:
      if (save_data == inv_data)
      {
        if (rc100_rx.data > 0)
        {
          rc100_rx.event_msg = rc100_rx.data;
        }
        else
        {
          rc100_rx.released_event = true;
        }
        rc100_rx.received = true;
        ret = true;
      }
      rc100_rx.state = 0;
      break;

    default:
      rc100_rx.state = 0;
      break;
  }

  return ret;
}
