/*******************************************************************************
  Copyright 2021 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl;


const uint8_t SERVO_SIZE = 8;           //Size of Servo PWM. 입력되는 Servo PWM의 갯수(변경X)



const float DXL_PROTOCOL_VERSION = 2.0; //Dynamixel Protocol Version. 다이나믹셀 프로토콜 버전
const uint32_t  DXL_BUADRATE = 57600;   //Dynamixel Baudrate(In this example, 57600bps has low error ratio.)

const uint8_t SERVO_PWM_INPUT_PIN[SERVO_SIZE]     = {0, 1, 4, 5, 6, 7, 8, 16};  //Pin Number of input PWM signal. PWM이 인가되는 핀의 번호

const uint8_t DXL_ID[SERVO_SIZE]                  = {1, 2, 3, 4, 5, 6, 7, 8};   //각 PWM에 대응하는 다이나믹셀 ID
const uint8_t SERVO_OP_MODE[SERVO_SIZE]           = {OP_VELOCITY, OP_POSITION, OP_POSITION, OP_POSITION,  //다이나믹셀의 동작 모드
                                                     OP_POSITION, OP_POSITION, OP_POSITION, OP_POSITION
                                                    };

const int8_t  SERVO_OP_DIRECTION[SERVO_SIZE]      = {1,     1,      1,      1,      1,      1,      1,      1};     //다이나믹셀의 회전 방향[-1=시계방향, 1=반시계방향]

const uint32_t DXL_IMIT_TIME_MS = 2000;   //다이나믹셀 Softstarting 시간[ms]
const uint32_t DXL_MODE_CENTOR[SERVO_SIZE]        = {0,     2048,   2048,   2048,   2048,   2048,   2048,   2048};  //PWM 변위 중간에 대응하는 위치[step] or 속도[step/s] 값
const uint32_t DXL_MOVE_RANGE[SERVO_SIZE]         = {200,   1024,   1024,   1024,   1024,   1024,   1024,   1024};  //PWM 변동 범위에 대응하는 위치[step] or 속도[step/s] 값

const float SERVO_PWM_MOVING_THRESHOLD[SERVO_SIZE] = {0.2,  0,      0,      0,      0,      0,      0,      0};     //PWM Duty에 대한 Threshold. Threshold가 넘어야 움직임


const float SERVO_PWM_CENTOR_TIME_MS[SERVO_SIZE]  = {1.5,   1.5,    1.5,    1.5,    1.5,    1.5,    1.5,    1.5};   //PWM 변위 중간에 대응하는 시간[ms]. 현재 PWM 변위 중간은 1.5ms
const float SERVO_PWM_RANGE_TIME_MS[SERVO_SIZE]   = {0.55,  0.55,   0.55,   0.55,   0.55,   0.55,   0.55,   0.55};  //PWM 변동 범위에 대응하는 시간[ms]. 현재 PWM 변위는 +-0.55ms



uint32_t _uiPWMStartTime[SERVO_SIZE];

const uint8_t FILTER_SIZE = 3;
uint32_t _uiPWMHighTime[SERVO_SIZE];

uint32_t _uiPWMRecvCount[SERVO_SIZE] = {0,};
uint32_t _uiPWMHighTime_Tmp[SERVO_SIZE][FILTER_SIZE];
uint8_t _ucInitFlag;

//This namespace is required to use Control table item names
using namespace ControlTableItem;


void setup() {

  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DXL_BUADRATE);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information
  for (int i = 0; i < SERVO_SIZE; i++) {

    dxl.ping(DXL_ID[i]);

    // Turn off torque when configuring items in EEPROM area.
    // 각 다이나믹셀의 동작 모드 변경
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], SERVO_OP_MODE[i]);
    dxl.torqueOn(DXL_ID[i]);


    // Set Interrupt to send PWM and checking Duty.
    // PWM 캡쳐를 위한 외부 인터럽트핀 설정
    pinMode(SERVO_PWM_INPUT_PIN[i], INPUT_PULLDOWN);
  }

  // Enable interrupt
  // PWM 캡쳐를 위한 외부 인터럽트핀 설정
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[0]), Pin0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[1]), Pin1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[2]), Pin2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[3]), Pin3_ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[4]), Pin4_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[5]), Pin5_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[6]), Pin6_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[7]), Pin7_ISR, CHANGE);

  delay(2000);

  // Wait
  DEBUG_SERIAL.println("Start Program.");
}


void loop() {

  //대기 모드인 경우
  if (_ucInitFlag == 0) {
    //첫 PWM이 인가되면 위치 초기화를 진행함.
    //초기화 = 현재위치에서 PWM에 대응하는 목표위치로 천천히 이동
     if (_uiPWMStartTime[0] != 0) {
      InitDXLPosition();
    }
  }
  //동작 모드인 경우
  else {

    // Check last PWM Rising time
    //마지막 PWM이 들어온 시간과 현재 시간을 비교하여 100ms이상이 차이나면 연결이 끊겼다고 판단.
    if( (micros() - _uiPWMStartTime[0]) > 100000){

      //여러번 검출 확인(오버플로우에 의해 생길 수 있으므로)
      _ucInitFlag++;
      if(_ucInitFlag > 2){

        //속도 모드로 동작하는 다이나믹셀은 멈추도록 함.(명령이 없다면 계속 회전함)
        for (int i = 0; i < SERVO_SIZE; i++){
            if(SERVO_OP_MODE[i] == OP_VELOCITY){
              dxl.setGoalVelocity(DXL_ID[i], 0);
            }
  
            _uiPWMStartTime[i] = 0;
        }
  
        //대기 모드로 변경
        _ucInitFlag = 0;
      }
    }
    else{
      for (int i = 0; i < SERVO_SIZE; i++) {

        //Threshold가 반영된 duty를 얻어옴.
        float fDuty = GetThresholdPWMDuty(i);

        // Position mode
        if (SERVO_OP_MODE[i] == OP_POSITION) {
  
          // Update goal position.
          uint32_t uiDxlGoalPosition = DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i] * (SERVO_OP_DIRECTION[i] * fDuty);
  
          // Check goal position is over limitation.
          // 목표위치가 동작 범위를 넘는지 확인
          if (uiDxlGoalPosition > (DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i])){
            uiDxlGoalPosition = DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i];
          }
          else if (uiDxlGoalPosition < (DXL_MODE_CENTOR[i] - DXL_MOVE_RANGE[i])){
            uiDxlGoalPosition = (DXL_MODE_CENTOR[i] - DXL_MOVE_RANGE[i]);
          }
  
          // Send goal position to Dynamixel.
          dxl.setGoalPosition(DXL_ID[i], uiDxlGoalPosition);
  
        }
        // Velocity mode
        else if (SERVO_OP_MODE[i] == OP_VELOCITY) {
  
          // Send goal position to Dynamixel.
          int32_t iDxlGoalVelocity = DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i] * (SERVO_OP_DIRECTION[i] * fDuty);
          dxl.setGoalVelocity(DXL_ID[i], iDxlGoalVelocity);
        }//end if
      }//end for
      
    }
  }
}


//PWM 캡쳐 외부 인터럽트 핸들러
void Pin0_ISR() {

  UpdateDuty(0);
}


void Pin1_ISR() {

  UpdateDuty(1);
}


void Pin2_ISR() {
  UpdateDuty(2);
}


void Pin3_ISR() {
  UpdateDuty(3);
}


void Pin4_ISR() {
  UpdateDuty(4);
}


void Pin5_ISR() {
  UpdateDuty(5);
}


void Pin6_ISR() {
  UpdateDuty(6);
}


void Pin7_ISR() {
  UpdateDuty(7);
}


// PWM 신호가 High인 시간을 얻는 함수
void UpdateDuty(uint8_t ucNum) {

  // Check PWM signal is Falling.
  if (digitalRead(SERVO_PWM_INPUT_PIN[ucNum]) == LOW) {

    // Calculate Duty of PWM Signal.
    _uiPWMHighTime_Tmp[ucNum][(_uiPWMRecvCount[ucNum]%FILTER_SIZE)] = micros() - _uiPWMStartTime[ucNum];
    _uiPWMRecvCount[ucNum]++;
    
  }
  else if (digitalRead(SERVO_PWM_INPUT_PIN[ucNum]) == HIGH) {

    // Recode time when PWM signal is going High
    _uiPWMStartTime[ucNum] = micros();
  }
}


//다이나믹셀을 목표위치로 천천히 이동하도록 하는 초기화 함수
void InitDXLPosition(){
  
  uint32_t uiPresentPosition[SERVO_SIZE] = {0,};
  _ucInitFlag = 1;

  dxl.ledOn(0xFE);

  //다이나믹셀의 현재 위치를 얻어옴.
  for (int i = 0; i < SERVO_SIZE; i++) {
    uiPresentPosition[i] = dxl.getPresentPosition(DXL_ID[i]);
  }
  
  // Go Init Position
  // Soft Start

  uint32_t uiInitStartTime = millis();
  float fT = 0;
  do{
    for (int i = 0; i < SERVO_SIZE; i++) {

      fT = (millis() - uiInitStartTime);

      //현재 PWM의 duty를 가져옴.
      float duty = GetPWMDuty(i);

      //목표 위치 계산
      uint32_t uiDxlGoalPosition = uiPresentPosition[i] + (DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i] * SERVO_OP_DIRECTION[i] * duty  - uiPresentPosition[i]) * (fT / DXL_IMIT_TIME_MS);
  
      // Send goal position to Dynamixel
      //위치 모드인 경우 목표 위치 전송
      //속도 모드인 경우 목표 속도로 0을 전송
      if (SERVO_OP_MODE[i] == OP_POSITION) {
        dxl.setGoalPosition(DXL_ID[i], uiDxlGoalPosition);
      }
      else if(SERVO_OP_MODE[i] == OP_VELOCITY){
        dxl.setGoalVelocity(DXL_ID[i], 0);
      }
    }
    
  }while(fT < DXL_IMIT_TIME_MS);
        
  dxl.ledOff(0xFE);
}


//3개의 PWM신호의 High 시간 샘플을 가져와 중간 값을 반환하는 함수
//시간 샘플의 지터 제거
uint32_t GetFiltedPWMTime(uint8_t ucNum){

  uint32_t uiMinTime = 0xFFFFFFFF, uiMaxTime = 0;
  uint32_t uiPWMHighTime[FILTER_SIZE] = {0,};
  uint32_t uiPWMTime = 0;
  
  for (register int j = 0; j < FILTER_SIZE; ++j) {
  
    uiPWMHighTime[j] = _uiPWMHighTime_Tmp[ucNum][j];
  
    if (uiMinTime > uiPWMHighTime[j]) {
      uiMinTime = uiPWMHighTime[j];
    }
    if (uiMaxTime < uiPWMHighTime[j]) {
      uiMaxTime = uiPWMHighTime[j];
    }

    uiPWMTime += uiPWMHighTime[j];
  }

  uiPWMTime = (uiPWMTime - uiMinTime - uiMaxTime) / (FILTER_SIZE-2);

  return uiPWMTime;
}


//PWM신호의 High 시간에 대응하는 Duty 반환하는 함수
float GetPWMDuty(uint8_t ucNum){
  return ((GetFiltedPWMTime(ucNum) / 1000.0) - SERVO_PWM_CENTOR_TIME_MS[ucNum]) / SERVO_PWM_RANGE_TIME_MS[ucNum];
}


//Threshold가 반영된 Duty를 반환하는 함수
float GetThresholdPWMDuty(uint8_t ucNum){

  // Calculate PWM Duty
  float fDuty = GetPWMDuty(ucNum);

  // Set Moving Threshold
  if (fDuty > 0) {
    if (fDuty > 1) fDuty = 1;
    fDuty = (fDuty < SERVO_PWM_MOVING_THRESHOLD[ucNum]) ? 0 : ((fDuty - SERVO_PWM_MOVING_THRESHOLD[ucNum]) / (1 - SERVO_PWM_MOVING_THRESHOLD[ucNum]));
  }
  else {
    if (fDuty < -1) fDuty = -1;
    fDuty = (fDuty > -SERVO_PWM_MOVING_THRESHOLD[ucNum]) ? 0 : ((fDuty + SERVO_PWM_MOVING_THRESHOLD[ucNum]) / (1 - SERVO_PWM_MOVING_THRESHOLD[ucNum]));
  }

  return fDuty;
}
