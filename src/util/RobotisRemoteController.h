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

#ifndef RC100_H_
#define RC100_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#include <SoftwareSerial.h>
#endif

////////// define RC-100 button key value ////////////////
#define RC100_BTN_U		(1)
#define RC100_BTN_D		(2)
#define RC100_BTN_L		(4)
#define RC100_BTN_R		(8)
#define RC100_BTN_1		(16)
#define RC100_BTN_2		(32)
#define RC100_BTN_3		(64)
#define RC100_BTN_4		(128)
#define RC100_BTN_5		(256)
#define RC100_BTN_6		(512)

#define PACKET_LENGTH 		6

class RobotisRemoteController : public Stream{
  public:
#ifdef SoftwareSerial_h  
    RobotisRemoteController(uint8_t rx_pin=7, uint8_t tx_pin=8);
#endif    
    RobotisRemoteController(HardwareSerial& port);
    virtual ~RobotisRemoteController();

    /**
     * @brief Initialization function to start communication with RC.
     *      Or change baudrate of baud serial port.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * rc.begin(115200);
     * @endcode
     * @param baud The port speed you want on the board (the speed to communicate with RC) (default : 57600)
     */    
    void begin(uint32_t baudrate = 57600);

    /**
     * @brief Check whether there is an RC packet normally received.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableData()){
     *   Serial.println(rc.readData());
     * }
     * @endcode
     * @return True(1) if it exists, false(0) otherwise.
     */    
    bool availableData(void);

    /**
     * @brief Read data of RC packet.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableData()){
     *   Serial.println(rc.readData());
     * }
     * @endcode
     * @return data of RC packet.
     */    
    uint16_t readData(void);

    /**
     * @brief Check whether there is an RC packet normally received.
     *       However, this only works after receiving the release event packet (data: 0).
     *       That is, it checks whether there is a packet when the button is released and then pressed again on the RC100 controller.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableEvent()){
     *   Serial.println(rc.readEvent());
     * }
     * @endcode
     * @return True(1) if it exists, false(0) otherwise.
     */   
    bool availableEvent(void);

    /**
     * @brief Read data of RC packet. (Use with availableEvent() function)
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableEvent()){
     *   Serial.println(rc.readEvent());
     * }
     * @endcode
     * @return data of RC packet.
     */        
    uint16_t readEvent(void);

    /**
     * @brief Clear Serial RX buffer.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * rc.flushRx();
     * @endcode
     */
    void flushRx(void);

    // Stream
    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    
    // Print
    virtual void flush() override;
    virtual size_t write(uint8_t) override;
    using Print::write; // pull in write(str) and write(buf, size) from Print

   
  private:
    typedef struct
    {
      uint8_t  state;
      uint8_t  index;
      bool     received;
      bool     released_event;
      uint16_t data;
      uint16_t event_msg;
    } rc100_t;

    rc100_t rc100_rx_;

#ifdef SoftwareSerial_h
    SoftwareSerial *p_sw_port_;
#endif
    HardwareSerial *p_hw_port_;
    Stream *stream_port_;

    bool enable_hw_port_;

    bool rc100Update(uint8_t data);
    bool rc100Receive(unsigned char *pPacket, int numPacket);
};

#endif /* RC100_H_ */
