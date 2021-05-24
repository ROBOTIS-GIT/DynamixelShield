// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Example Environment
//
// - DYNAMIXEL: X series (except XL-320)
//              ID = 1 ~ 7, Baudrate = 1000000bps, Protocol 2.0
// - Controller: Arduino MKR ZERO
//               DYNAMIXEL Shield for Arduino MKR
// - RC Controller: Futaba T10J
// - RC Receiver: Futaba R3008SB
// - https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/#examples
//
// Author: Seungmin Lee

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);  // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl;

// Number of PWM Channels
const uint8_t PWM_CHANNEL_SIZE = 8;
// DYNAMIXEL Protocol Version [1.0 / 2.0]
const float DXL_PROTOCOL_VERSION = 2.0;
// DYNAMIXEL Baudrate
const uint32_t DXL_BUADRATE = 1000000;
// Assign MKR Board pins for receiving PWM signals from the RC Receiver
const uint8_t MKR_PWM_INPUT_PINS[PWM_CHANNEL_SIZE] = {0, 1, 4, 5, 6, 7, 8, 16};
// DYNAMIXEL ID for each PWM channel
const uint8_t DXL_ID[PWM_CHANNEL_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
// Operating Mode of each DYNAMIXEL
const uint8_t SERVO_OP_MODE[PWM_CHANNEL_SIZE] = {
  OP_VELOCITY, OP_POSITION, OP_POSITION, OP_POSITION,
  OP_POSITION, OP_POSITION, OP_POSITION, OP_POSITION
};

// Rotation direction of DYNAMIXEL [-1:CW / 1:CCW]
const int8_t ROTATION_DIRECTION[PWM_CHANNEL_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1};
// DYNAMIXEL Soft Start duration in millisecond
const uint32_t SOFT_START_DURATION = 2000;
// Home Position[step] or Speed[step/s] within the PWM range
const uint32_t HOME_PWM_VALUE[PWM_CHANNEL_SIZE] = {0, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
// Absolute maximum value of Position[step] or Speed[step/s] within the PWM range
const uint32_t MAX_PWM_VALUE[PWM_CHANNEL_SIZE] = {200, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
// Threshold value of the PWM Duty to operate DYNAMIXEL
const float PWM_THRESHOLD[PWM_CHANNEL_SIZE] = {0.2, 0, 0, 0, 0, 0, 0, 0};
// The median time of PWM signal in millisecond. The default median is set to 1.5ms
const float MEDIAN_TIME_OF_PWM_RANGE[PWM_CHANNEL_SIZE] = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
// The absolute maximum time of PWM range in millisecond. The default PWM range is +-0.55ms
const float TIME_RANGE_OF_PWM[PWM_CHANNEL_SIZE] = {0.55, 0.55, 0.55, 0.55, 0.55, 0.55, 0.55, 0.55};
const uint8_t FAIL_SAFE_ENABLED = 1;  // Activate the Fail Safe
// Enter to Init mode when there's no PWM signal within this timer period (in microsecond)
const uint32_t FAIL_SAFE_WATCHDOG_TIMER = 250000;
// Home positions for Fail Safe mode[step]
const uint32_t FAIL_SAFE_HOME_POSITION[PWM_CHANNEL_SIZE] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
const uint8_t FILTER_SIZE = 3;

volatile uint32_t latest_pwm_timestamp[PWM_CHANNEL_SIZE];
volatile uint32_t count_received_pwm[PWM_CHANNEL_SIZE] = {0};
volatile uint32_t pwm_high_time_buffer[PWM_CHANNEL_SIZE][FILTER_SIZE];
uint8_t init_flag, fail_safe_count;

void setup()
{
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DXL_BUADRATE);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information
  for (int i = 0; i < PWM_CHANNEL_SIZE; i++) {
    dxl.ping(DXL_ID[i]);

    // Turn off torque when configuring items in EEPROM area.
    dxl.torqueOff(DXL_ID[i]);
    // Set operating mode of each DYNAMIXEL
    dxl.setOperatingMode(DXL_ID[i], SERVO_OP_MODE[i]);
    dxl.torqueOn(DXL_ID[i]);

    // Set Interrupts to read PWM
    pinMode(MKR_PWM_INPUT_PINS[i], INPUT_PULLDOWN);
  }

  // Set external interrupt to capture PWM signal
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[0]), Pin0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[1]), Pin1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[2]), Pin2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[3]), Pin3_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[4]), Pin4_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[5]), Pin5_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[6]), Pin6_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[7]), Pin7_ISR, CHANGE);

  delay(2000);
  DEBUG_SERIAL.println("Start Program.");
}

void loop()
{
  // standby mode
  if (init_flag == 0) {
    // Initialize when the first PWM is received
      if (latest_pwm_timestamp[0] != 0) {
        init_dynamixel_position();
    }
  } else {  // when under operation
    // Check last PWM Rising time
    // Compare the last PWM received time with the fail safe watchdog timer
    if (FAIL_SAFE_ENABLED && ((micros() - latest_pwm_timestamp[0]) > FAIL_SAFE_WATCHDOG_TIMER)) {
      // Initiate Fail Safe procedure
      fail_safe_count++;

      if (fail_safe_count > 1) {
        fail_safe_count = 0;

        // Move to home position or set to 0 RPM
        for (int i = 0; i < PWM_CHANNEL_SIZE; i++) {
          if(SERVO_OP_MODE[i] == OP_POSITION) {
            dxl.setGoalPosition(DXL_ID[i], FAIL_SAFE_HOME_POSITION[i]);
          } else if (SERVO_OP_MODE[i] == OP_VELOCITY) {
            dxl.setGoalVelocity(DXL_ID[i], 0);
          }
          latest_pwm_timestamp[i] = 0;
        }

        // return to standby mode
        init_flag = 0;
      }
    } else {
      fail_safe_count = 0;
      uint32_t goal_value[PWM_CHANNEL_SIZE] = {0};

      for (int i = 0; i < PWM_CHANNEL_SIZE; i++) {
        float threshold_applied_pwm_duty = get_threshold_pwm_duty(i);

        if (SERVO_OP_MODE[i] == OP_POSITION) {
          // Update the goal position
          goal_value[i] = HOME_PWM_VALUE[i] + MAX_PWM_VALUE[i] * (ROTATION_DIRECTION[i] * threshold_applied_pwm_duty);

          // Check the goal position is in valid range
          if (goal_value[i] > (HOME_PWM_VALUE[i] + MAX_PWM_VALUE[i])) {
            goal_value[i] = HOME_PWM_VALUE[i] + MAX_PWM_VALUE[i];
          } else if (goal_value[i] < (HOME_PWM_VALUE[i] - MAX_PWM_VALUE[i])) {
            goal_value[i] = (HOME_PWM_VALUE[i] - MAX_PWM_VALUE[i]);
          }
        } else if (SERVO_OP_MODE[i] == OP_VELOCITY) {
          goal_value[i] = HOME_PWM_VALUE[i] + MAX_PWM_VALUE[i] * (ROTATION_DIRECTION[i] * threshold_applied_pwm_duty);
        }
      }

      // Write the goal value to DYNAMIXEL
      bulk_write_goal_data(goal_value);
    }
  }
}

// External interrupt handlers for capturing PWM signal
void Pin0_ISR()
{
  update_pwm_duty(0);
}

void Pin1_ISR()
{
  update_pwm_duty(1);
}

void Pin2_ISR()
{
  update_pwm_duty(2);
}

void Pin3_ISR()
{
  update_pwm_duty(3);
}

void Pin4_ISR()
{
  update_pwm_duty(4);
}

void Pin5_ISR()
{
  update_pwm_duty(5);
}

void Pin6_ISR()
{
  update_pwm_duty(6);
}

void Pin7_ISR()
{
  update_pwm_duty(7);
}

void update_pwm_duty(uint8_t ucNum)
{
  // When PWM signal is falling
  if (digitalRead(MKR_PWM_INPUT_PINS[ucNum]) == LOW) {
    // Calculate Duty of PWM Signal.
    pwm_high_time_buffer[ucNum][(count_received_pwm[ucNum]%FILTER_SIZE)] =
      micros() - latest_pwm_timestamp[ucNum];

    count_received_pwm[ucNum]++;
  } else if (digitalRead(MKR_PWM_INPUT_PINS[ucNum]) == HIGH) {  // When PWM signal is rising
    // Recode time when PWM signal is going High
    latest_pwm_timestamp[ucNum] = micros();
  }
}

// slowly move to the PWM position
void init_dynamixel_position()
{
  uint32_t present_position_buffer[PWM_CHANNEL_SIZE] = {0};
  init_flag = 1;
  dxl.ledOn(DXL_BROADCAST_ID);

  DEBUG_SERIAL.println("Read DYNAMIXEL Present Position");
  sync_read_present_position(present_position_buffer);

  // Soft Start to Init Position
  uint32_t init_start_timestamp = millis();
  float elapsed_time = 0;

  do {
    uint32_t goal_value[PWM_CHANNEL_SIZE] = {0};
    elapsed_time = (millis() - init_start_timestamp);

    for (int i = 0; i < PWM_CHANNEL_SIZE; i++) {
      float duty = get_pwm_duty(i);

      // Calculate Goal Velocity or Position values
      if (SERVO_OP_MODE[i] == OP_VELOCITY) {
        goal_value[i] = 0;
      } else {
        goal_value[i] = present_position_buffer[i] +
          (HOME_PWM_VALUE[i] + MAX_PWM_VALUE[i] * ROTATION_DIRECTION[i] * duty - present_position_buffer[i]) *
          (elapsed_time / SOFT_START_DURATION);
      }
    }

    bulk_write_goal_data(goal_value);
  } while (elapsed_time < SOFT_START_DURATION);

  dxl.ledOff(DXL_BROADCAST_ID);
}

// Sampling 3 PWM and return the median value
// Filter the jitter in PWM time
uint32_t get_filtered_pwm_time(uint8_t ucNum)
{
  uint32_t min_time = 0xFFFFFFFF, max_time = 0;
  uint32_t pwm_high_time[FILTER_SIZE] = {0};
  uint32_t pwm_time = 0;

  for (register int j = 0; j < FILTER_SIZE; ++j) {
    pwm_high_time[j] = pwm_high_time_buffer[ucNum][j];

    if (min_time > pwm_high_time[j]) {
      min_time = pwm_high_time[j];
    }
    if (max_time < pwm_high_time[j]) {
      max_time = pwm_high_time[j];
    }

    pwm_time += pwm_high_time[j];
  }

  pwm_time = (pwm_time - min_time - max_time) / (FILTER_SIZE - 2);
  return pwm_time;
}

// Return PWM duty (PWM high time)
float get_pwm_duty(uint8_t ucNum)
{
  return ((get_filtered_pwm_time(ucNum) / 1000.0) - MEDIAN_TIME_OF_PWM_RANGE[ucNum]) / TIME_RANGE_OF_PWM[ucNum];
}

// Return PWM duty with Threshold
float get_threshold_pwm_duty(uint8_t ucNum)
{
  // Calculate PWM Duty
  float pwm_duty = get_pwm_duty(ucNum);

  // Set Moving Threshold
  if (pwm_duty > 0) {
    if (pwm_duty > 1) {
      pwm_duty = 1;
    }
    pwm_duty = (pwm_duty < PWM_THRESHOLD[ucNum]) ?
      0 : ((pwm_duty - PWM_THRESHOLD[ucNum]) / (1 - PWM_THRESHOLD[ucNum]));
  } else {
    if (pwm_duty < -1) {
      pwm_duty = -1;
    }
    pwm_duty = (pwm_duty > -PWM_THRESHOLD[ucNum]) ?
      0 : ((pwm_duty + PWM_THRESHOLD[ucNum]) / (1 - PWM_THRESHOLD[ucNum]));
  }
  return pwm_duty;
}

void bulk_write_goal_data(uint32_t* goal_value)
{
  ParamForBulkWriteInst_t bulk_write_param;

  // fill the members of structure for bulkWrite
  for (int i = 0; i < PWM_CHANNEL_SIZE; ++i) {
    bulk_write_param.xel[i].id = DXL_ID[i];
    bulk_write_param.xel[i].length = 4;
    bulk_write_param.xel[i].addr = (SERVO_OP_MODE[i] == OP_POSITION) ? 116 : 104;
    memcpy(bulk_write_param.xel[i].data, (uint8_t*)&goal_value[i], 4);
  }

  bulk_write_param.id_count = PWM_CHANNEL_SIZE;
  dxl.bulkWrite(bulk_write_param);
}

void sync_read_present_position(uint32_t* present_position)
{
  ParamForSyncReadInst_t sync_read_param;
  RecvInfoFromStatusInst_t read_result;

  // Create a structure for Sync Read
  sync_read_param.addr = 132;  // Control Table Address of Present Position
  sync_read_param.length = 4;  // Data Length in byte of Present Position
  sync_read_param.id_count = PWM_CHANNEL_SIZE;

  for (uint8_t i = 0; i < PWM_CHANNEL_SIZE; ++i) {
    sync_read_param.xel[i].id = DXL_ID[i];
  }

  // Disable the external interrupt while receiving data from DYNAMIXEL
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[0]));
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[1]));
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[2]));
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[3]));
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[4]));
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[5]));
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[6]));
  detachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[7]));

  // Transmit the Sync Read Instruction to DYNAMIXEL
  dxl.syncRead(sync_read_param, read_result);

  // Read the Present Position from received Status Packet
  for (uint8_t i = 0; i < PWM_CHANNEL_SIZE; ++i) {
    memcpy(&present_position[i], read_result.xel[i].data, 4);
  }

  // Enable the external interrup.
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[0]), Pin0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[1]), Pin1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[2]), Pin2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[3]), Pin3_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[4]), Pin4_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[5]), Pin5_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[6]), Pin6_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MKR_PWM_INPUT_PINS[7]), Pin7_ISR, CHANGE);

  // Allow some delay while activating the RC receiver
  delay(1000);
}
