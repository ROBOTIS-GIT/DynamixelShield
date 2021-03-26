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


const uint8_t SERVO_SIZE = 8; // The number of input Servo PWM.


// Define the DYNAMIXEL Protocol to use.
const float DXL_PROTOCOL_VERSION = 2.0; 

// Define the baud rate of DYNAMIXEL in use. (In this example, 57600bps has a low error ratio.)
const uint32_t  DXL_BUADRATE = 1000000; 

// Pin Number of input PWM signal. 
const uint8_t SERVO_PWM_INPUT_PIN[SERVO_SIZE]     = {0, 1, 4, 5, 6, 7, 8, 16};

// DYNAMIXEL ID corresponding to each PWM signal. 
const uint8_t DXL_ID[SERVO_SIZE]                  = {1, 2, 3, 4, 5, 6, 7, 8};

// Define the Operating Mode of DYNAMIXEL.
const uint8_t SERVO_OP_MODE[SERVO_SIZE]           = {OP_VELOCITY, OP_POSITION, OP_POSITION, OP_POSITION,
                                                     OP_POSITION, OP_POSITION, OP_POSITION, OP_POSITION
                                                    }; 
// DYNAMIXEL’s moving direction (-1: CW, 1 : CCW)
const int8_t  SERVO_OP_DIRECTION[SERVO_SIZE]      = {1,     1,      1,      1,      1,      1,      1,      1};


// The time [ms] of the soft start of DYNAMIXEL
const uint32_t DXL_IMIT_TIME_MS = 2000;

// The DYNAMIXEL position [step] or velocity [step/s] corresponding to its middle of PWM displacement.
const uint32_t DXL_MODE_CENTOR[SERVO_SIZE]        = {0,     2048,   2048,   2048,   2048,   2048,   2048,   2048};

// DYNAMIXEL position [step] or velocity [step/s] corresponding to the range of PWM change. 
const uint32_t DXL_MOVE_RANGE[SERVO_SIZE]         = {200,   1024,   1024,   1024,   1024,   1024,   1024,   1024};

// The value of threshold corresponding to PWM duty. Note that DYNAMIXEL moves when over the set threshold only.
const float SERVO_PWM_MOVING_THRESHOLD[SERVO_SIZE] = {0.2,  0,      0,      0,      0,      0,      0,      0};

// Time [ms] corresponding to the middle of PWM displacement. The middle of current PWM displacement is 1.5 ms.
const float SERVO_PWM_CENTOR_TIME_MS[SERVO_SIZE]  = {1.5,   1.5,    1.5,    1.5,    1.5,    1.5,    1.5,    1.5};

// Time [ms] to the range of the PWM change. The current PWM displacement is +-0.55 ms. 
const float SERVO_PWM_RANGE_TIME_MS[SERVO_SIZE]   = {0.55,  0.55,   0.55,   0.55,   0.55,   0.55,   0.55,   0.55};

// Activating the Fail Safe feature. 
const uint8_t FAIL_SAFE_ENABLE = 1;

// Turning to Init mode if no PWM input signal over the time criteria [500 ms].
const uint32_t FAIL_SAFE_TIME_MS = 500;

// Target goal position [step] when Fail Safe. 
const uint32_t DXL_FAIL_SAFE_POS[SERVO_SIZE]      = {2048,   2048,   2048,   2048,   2048,   2048,   2048,   2048};


volatile uint32_t _uiPWMStartTime[SERVO_SIZE];

const uint8_t FILTER_SIZE = 3;
uint32_t _uiPWMHighTime[SERVO_SIZE];

volatile uint32_t _uiPWMRecvCount[SERVO_SIZE] = {0,};
volatile uint32_t _uiPWMHighTime_Tmp[SERVO_SIZE][FILTER_SIZE];
uint8_t _ucInitFlag, _ucInitCnt;

// The required namespace to use the items in the control table of DYNAMIXEL.
using namespace ControlTableItem;


void setup() {

  // For Arduino Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. Make sure to use the same baudrate as the set baudrate of DYNAMIXEL.
  dxl.begin(DXL_BUADRATE);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information.
  for (int i = 0; i < SERVO_SIZE; i++) {

    dxl.ping(DXL_ID[i]);

    // Turn off torque when configuring items in EEPROM area.
    // Set the DYNAMIXEL’s Operating Mode. 
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], SERVO_OP_MODE[i]);
    dxl.torqueOn(DXL_ID[i]);

    // Set Interrupt to send PWM and check Duty.
    pinMode(SERVO_PWM_INPUT_PIN[i], INPUT_PULLDOWN);
  }

  // Enable interrupt
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

  // In standby mode
  if (_ucInitFlag == 0) {
    
    // Position initialization by the first input PWM signal. 
    
    // Initialization = Slow moving from the current position to the goal position set by the PWM signal.
     if (_uiPWMStartTime[0] != 0) {
      InitDXLPosition();
    }
  }
  
  // In Operating Mode 
  else {

    // Check last PWM Rising time
    // Fail Safe.
    // Over 100 ms delay between the last PWM rising time and the present time will cause the communication fail.
    if(FAIL_SAFE_ENABLE && ((micros() - _uiPWMStartTime[0]) > (FAIL_SAFE_TIME_MS * 500))){

      // Error detection in multiple times.
      _ucInitCnt++;
      if(_ucInitCnt > 1){

        _ucInitCnt = 0;

        // DYNAMIXELs in Position Control mode moves to the set Goal Position. 
        // DYNAMIXELs in Velocity Mode will stop moving; If no command, DYNAMIXEL will keep moving. 
        for (int i = 0; i < SERVO_SIZE; i++){
            if(SERVO_OP_MODE[i] == OP_POSITION){
              dxl.setGoalPosition(DXL_ID[i], DXL_FAIL_SAFE_POS[i]);
            }
            else if(SERVO_OP_MODE[i] == OP_VELOCITY){
              dxl.setGoalVelocity(DXL_ID[i], 0);
            }
  
            _uiPWMStartTime[i] = 0;
        }
  
        // Switch to Standby mode.
        _ucInitFlag = 0;
      }
    }
    else{
      _ucInitCnt = 0;
      uint32_t uiDxlGoal[SERVO_SIZE] = {0, };
      for (int i = 0; i < SERVO_SIZE; i++) {

        // Get duty reflected by the threshold.
        float fDuty = GetThresholdPWMDuty(i);

        // Position mode
        if (SERVO_OP_MODE[i] == OP_POSITION) {
  
          // Update Goal Position.
          uiDxlGoal[i] = DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i] * (SERVO_OP_DIRECTION[i] * fDuty);
  
          // Check if the set Goal Position is out of its operating range.
          if (uiDxlGoal[i] > (DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i])){
            uiDxlGoal[i] = DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i];
          }
          else if (uiDxlGoal[i] < (DXL_MODE_CENTOR[i] - DXL_MOVE_RANGE[i])){
            uiDxlGoal[i] = (DXL_MODE_CENTOR[i] - DXL_MOVE_RANGE[i]);
          }
  
          // Send Goal Position to DYNAMIXEL.
          // dxl.setGoalPosition(DXL_ID[i], uiDxlGoal[i]);
  
        }
        // Velocity mode
        else if (SERVO_OP_MODE[i] == OP_VELOCITY) {
  
          // Send Goal Position to DYNAMIXEL.
          uiDxlGoal[i] = DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i] * (SERVO_OP_DIRECTION[i] * fDuty);
          //dxl.setGoalVelocity(DXL_ID[i], uiDxlGoal[i]);
        } 
      }

      BulkWriteGoalData(uiDxlGoal);
    }
  }
  
}


// External interrupt handler of PWM capture.
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


// Function to get the time when PWM signal is HIGH.
void UpdateDuty(uint8_t ucNum) {

  // Check if PWM signal is Falling.
  if (digitalRead(SERVO_PWM_INPUT_PIN[ucNum]) == LOW) {

    // Calculate Duty of PWM Signal.
    _uiPWMHighTime_Tmp[ucNum][(_uiPWMRecvCount[ucNum]%FILTER_SIZE)] = micros() - _uiPWMStartTime[ucNum];
    _uiPWMRecvCount[ucNum]++;
    
  }
  else if (digitalRead(SERVO_PWM_INPUT_PIN[ucNum]) == HIGH) {

    // Recode the time when PWM signal is going High
    _uiPWMStartTime[ucNum] = micros();
  }
}


// Init function to let DYNAMIXEL move to the set Goal Position slowly. 
void InitDXLPosition(){
  
  uint32_t uiPresentPosition[SERVO_SIZE] = {0,};
  _ucInitFlag = 1;

  dxl.ledOn(0xFE);

  // Get the Present Position of the DYNAMIXEL. 
  //for (int i = 0; i < SERVO_SIZE; i++) {
  //  uiPresentPosition[i] = dxl.getPresentPosition(DXL_ID[i]);
  //}
  DEBUG_SERIAL.println("Read PrePos");
  SyncReadPresentPosition(uiPresentPosition);
  
  // Go Init Position
  // Soft Start
  uint32_t uiInitStartTime = millis();
  float fT = 0;
  do{
    uint32_t uiDxlGoal[SERVO_SIZE] = {0, };
    fT = (millis() - uiInitStartTime);
    for (int i = 0; i < SERVO_SIZE; i++) {

      // Get duty of the current PWM.
      float duty = GetPWMDuty(i);

      // Calculate the Goal Position.
      if(SERVO_OP_MODE[i] == OP_VELOCITY)
        uiDxlGoal[i] = 0;
      else
        uiDxlGoal[i] = uiPresentPosition[i] + (DXL_MODE_CENTOR[i] + DXL_MOVE_RANGE[i] * SERVO_OP_DIRECTION[i] * duty  - uiPresentPosition[i]) * (fT / DXL_IMIT_TIME_MS);
    }
    BulkWriteGoalData(uiDxlGoal);
    
  }while(fT < DXL_IMIT_TIME_MS);
        
  dxl.ledOff(0xFE);
}


// Function to return the middle value of the HIGH time samples of three PWM signals.
// Rid the jittering of the time samples.
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


// Function to return duty corresponding to the HIGH time duration of PWM signal.
float GetPWMDuty(uint8_t ucNum){
  return ((GetFiltedPWMTime(ucNum) / 1000.0) - SERVO_PWM_CENTOR_TIME_MS[ucNum]) / SERVO_PWM_RANGE_TIME_MS[ucNum];
}

// Function to return the duty reflected by the threshold. 
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


void BulkWriteGoalData(uint32_t* poGoal){

  ParamForBulkWriteInst_t bulk_write_param;

  // fill the members of structure for bulkWrite.
  for(int i=0; i<SERVO_SIZE; ++i){
    bulk_write_param.xel[i].id = DXL_ID[i];
    bulk_write_param.xel[i].length = 4;
    bulk_write_param.xel[i].addr = (SERVO_OP_MODE[i] == OP_POSITION) ? 116 : 104;

    memcpy(bulk_write_param.xel[i].data, (uint8_t*)&poGoal[i], 4);
  }
  bulk_write_param.id_count = SERVO_SIZE;

  dxl.bulkWrite(bulk_write_param);
}


void SyncReadPresentPosition(uint32_t* poPrePos){

  ParamForSyncReadInst_t sync_read_param;
  RecvInfoFromStatusInst_t read_result;

  // Generate a sync_read’s packet. 
  sync_read_param.addr = 132;//GetDxlTableID(tableIdx);
  sync_read_param.length = 4;
  sync_read_param.id_count = SERVO_SIZE;

  for(uint8_t i=0; i<SERVO_SIZE; ++i){
    sync_read_param.xel[i].id = DXL_ID[i];
  }

  // To prevent receiving data error, inactivate the external interrupt. 
  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[0]));
  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[1]));
  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[2]));
  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[3]));

  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[4]));
  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[5]));
  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[6]));
  detachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[7]));
  
  // Transmit the sync_read packet.
  dxl.syncRead(sync_read_param, read_result);
 
  // Get value of DYNAMIXEL from the read data of i-th
  for(uint8_t i=0; i<SERVO_SIZE; ++i){
    memcpy(&poPrePos[i], read_result.xel[i].data, 4);
  }

  // Back to activate the external interrupt. 
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[0]), Pin0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[1]), Pin1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[2]), Pin2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[3]), Pin3_ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[4]), Pin4_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[5]), Pin5_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[6]), Pin6_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO_PWM_INPUT_PIN[7]), Pin7_ISR, CHANGE);

  // Give the RC receiver the time to get ready for the operation. 
  delay(1000);
}
