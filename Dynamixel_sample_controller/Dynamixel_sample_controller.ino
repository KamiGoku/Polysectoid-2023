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

#include <DynamixelShield.h>
#include "Routine.h"
#define SEGMENT_NUMBER 5

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

uint8_t DXL_ID[] = {1,2,3,4,5,6,7,8,9,10};
const int number_Of_Motor = sizeof(DXL_ID) / sizeof(DXL_ID[0]);
const float DXL_PROTOCOL_VERSION = 2.0;

int8_t worm_pattern[][SEGMENT_NUMBER] = {{0,0,1,1,1},{1,0,0,1,1},{1,1,0,0,1},{1,1,1,0,0},{0,1,1,1,0}}; //1 means contract, 0 means relax
int8_t worm_pattern_turning[][SEGMENT_NUMBER] = {{1,1,0,-1,-1},{1,0,-1,-1,0},{0,-1,-1,0,1},{-1,-1,0,1,1},{-1,0,1,1,0},{0,1,1,0,-1}}; //1 means turning left. -1 means turning right, 0 means not turning

int32_t peristalsis_cycle_size = sizeof(worm_pattern) / sizeof(worm_pattern[0]);
int32_t undulation_cycle_size = sizeof(worm_pattern_turning) / sizeof(worm_pattern_turning[0]);

int iteration = 0;
int32_t calibration[number_Of_Motor];
const int32_t full_contraction = -630;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  for(int i = 0;i<number_Of_Motor;i++){ //iterate through all the motors
    dxl.ping(DXL_ID[i]);
    delay(800);
  
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_EXTENDED_POSITION);
    delay(800);
    dxl.torqueOn(DXL_ID[i]);
    delay(800);
    calibration[i] = (int32_t)dxl.getCurAngle(DXL_ID[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  
    
    // Print present position in raw value
    // DEBUG_SERIAL.print("Present Position(raw) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i]));
    // DEBUG_SERIAL.print("Present Position(raw) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i+1]));
    // delay(1000);
  
    // Set Goal Position in DEGREE value
    // dxl.setGoalPosition(DXL_ID[2*i], currentILEFTposition, UNIT_DEGREE);
    // dxl.setGoalPosition(DXL_ID[2*i+1], currentIRIGHTposition, UNIT_DEGREE);
    // delay(1000);
    // Print present position in degree value
    // DEBUG_SERIAL.print("Present Position(degree) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i], UNIT_DEGREE));
    // DEBUG_SERIAL.print("Present Position(degree) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i+1], UNIT_DEGREE));
    // delay(1000);

  peristalsisRoutine (dxl, worm_pattern, number_Of_Motor, calibration, DXL_ID, iteration, full_contraction);
  // undulationRoutine (dxl, worm_pattern_turning, number_Of_Motor, calibration, DXL_ID, iteration);

  iteration++;
  iteration = iteration % peristalsis_cycle_size;
  DEBUG_SERIAL.print("Iteration Number: ");
  DEBUG_SERIAL.println(iteration);
  delay(2000);

}
